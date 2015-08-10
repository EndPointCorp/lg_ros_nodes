from itertools import chain
from hashlib import sha1

import pulsar
from pulsar.utils.structures import mapping_iterator, Zset
from pulsar.utils.pep import native_str, zip, ispy3k, iteritems
from pulsar.apps.ds import COMMANDS_INFO

from .pubsub import PubSub


if ispy3k:
    str_or_bytes = (bytes, str)
else:   # pragma    nocover
    str_or_bytes = basestring

INVERSE_COMMANDS_INFO = dict(((i.method_name, i.name)
                              for i in COMMANDS_INFO.values()))


class CommandError(pulsar.PulsarException):
    pass


class Executor:
    __slots__ = ('client', 'command')

    def __init__(self, client, command):
        self.client = client
        self.command = command

    def __call__(self, *args, **options):
        return self.client.execute(self.command, *args, **options)


class ResponseError:
    __slots__ = ('exception',)

    def __init__(self, exception):
        self.exception = exception


def dict_merge(*dicts):
    merged = {}
    [merged.update(d) for d in dicts]
    return merged


def pairs_to_object(response, factory=None):
    it = iter(response)
    return (factory or dict)(zip(it, it))


def values_to_object(response, fields=None, factory=None):
    if fields is not None:
        return (factory or dict)(zip(fields, response))
    else:
        return response


def string_keys_to_dict(key_string, callback):
    return dict.fromkeys(key_string.split(), callback)


def parse_info(response):
    info = {}
    response = native_str(response)

    def get_value(value):
        if ',' not in value or '=' not in value:
            try:
                if '.' in value:
                    return float(value)
                else:
                    return int(value)
            except ValueError:
                return value
        else:
            sub_dict = {}
            for item in value.split(','):
                k, v = item.rsplit('=', 1)
                sub_dict[k] = get_value(v)
            return sub_dict

    for line in response.splitlines():
        if line and not line.startswith('#'):
            key, value = line.split(':', 1)
            info[key] = get_value(value)
    return info


def values_to_zset(response, withscores=False, **kw):
    if withscores:
        it = iter(response)
        return Zset(((float(score), value) for value, score in zip(it, it)))
    else:
        return response


def sort_return_tuples(response, groups=None, **options):
    """
    If ``groups`` is specified, return the response as a list of
    n-element tuples with n being the value found in options['groups']
    """
    if not response or not groups:
        return response
    return list(zip(*[response[i::groups] for i in range(groups)]))


def pubsub_callback(response, subcommand=None):
    if subcommand == 'numsub':
        it = iter(response)
        return dict(((k, int(v)) for k, v in zip(it, it)))
        return pairs_to_object(response)
    elif subcommand == 'numpat':
        return int(response)
    else:
        return response


class Consumer(pulsar.ProtocolConsumer):

    RESPONSE_CALLBACKS = dict_merge(
        string_keys_to_dict(
            'BGSAVE FLUSHALL FLUSHDB HMSET LSET LTRIM MSET RENAME RESTORE '
            'SAVE SELECT SHUTDOWN SLAVEOF SET WATCH UNWATCH',
            lambda r: r == b'OK'
        ),
        string_keys_to_dict('SORT', sort_return_tuples),
        string_keys_to_dict('BLPOP BRPOP', lambda r: r and tuple(r) or None),
        string_keys_to_dict('SMEMBERS SDIFF SINTER SUNION', set),
        string_keys_to_dict('ZINCRBY ZSCORE',
                            lambda v: float(v) if v is not None else v),
        string_keys_to_dict('ZRANGE ZRANGEBYSCORE ZREVRANGE ZREVRANGEBYSCORE',
                            values_to_zset),
        string_keys_to_dict('EXISTS EXPIRE EXPIREAT PEXPIRE PEXPIREAT '
                            'PERSIST RENAMENX',
                            lambda r: bool(r)),
        {
            'PING': lambda r: r == b'PONG',
            'PUBSUB': pubsub_callback,
            'INFO': parse_info,
            'TIME': lambda x: (int(float(x[0])), int(float(x[1]))),
            'HGETALL': pairs_to_object,
            'HINCRBYFLOAT': lambda r: float(r),
            'HMGET': values_to_object,
            'TYPE': lambda r: r.decode('utf-8')
        }
    )

    def start_request(self):
        conn = self._connection
        args = self._request[0]
        if len(self._request) == 2:
            chunk = conn.parser.pack_command(args)
        else:
            chunk = conn.parser.pack_pipeline(args)
        conn._transport.write(chunk)

    def parse_response(self, response, command, options):
        callback = self.RESPONSE_CALLBACKS.get(command.upper())
        return callback(response, **options) if callback else response

    def data_received(self, data):
        conn = self._connection
        parser = conn.parser
        parser.feed(data)
        response = parser.get()
        request = self._request
        if len(request) == 2:
            if response is not False:
                if not isinstance(response, Exception):
                    cmnd = request[0][0]
                    response = self.parse_response(response, cmnd, request[1])
                else:
                    response = ResponseError(response)
                self.finished(response)
        else:   # pipeline
            commands, raise_on_error, responses = request
            error = None
            while response is not False:
                if (isinstance(response, Exception) and raise_on_error
                        and not error):
                    error = response
                responses.append(response)
                response = parser.get()
            if len(responses) == len(commands):
                response = []
                for cmds, resp in zip(commands[1:-1], responses[-1]):
                    args, options = cmds
                    if isinstance(resp, Exception) and not error:
                        error = resp
                    resp = self.parse_response(resp, args[0], options)
                    response.append(resp)
                if error and raise_on_error:
                    response = ResponseError(error)
                self.finished(response)


class RedisClient(object):
    '''Client for :class:`.RedisStore`.

    .. attribute:: store

        The :class:`.RedisStore` for this client.
    '''
    def __init__(self, store):
        self.store = store

    def __repr__(self):
        return '%s(%s)' % (self.__class__.__name__, self.store)
    __str__ = __repr__

    def pubsub(self, **kw):
        return PubSub(self.store, **kw)

    def pipeline(self):
        '''Create a :class:`.Pipeline` for pipelining commands
        '''
        return Pipeline(self.store)

    def execute(self, command, *args, **options):
        return self.store.execute(command, *args, **options)
    execute_command = execute

    # special commands

    # STRINGS
    def decr(self, key, ammount=None):
        if ammount is None:
            return self.execute('decr', key)
        else:
            return self.execute('decrby', key, ammount)

    def incr(self, key, ammount=None):
        if ammount is None:
            return self.execute('incr', key)
        else:
            return self.execute('incrby', key, ammount)

    # HASHES
    def hmget(self, key, *fields):
        return self.execute('hmget', key, *fields, fields=fields)

    def hmset(self, key, iterable):
        args = []
        [args.extend(pair) for pair in mapping_iterator(iterable)]
        return self.execute('hmset', key, *args)

    # LISTS
    def blpop(self, keys, timeout=0):
        if timeout is None:
            timeout = 0
        if isinstance(keys, str_or_bytes):
            keys = [keys]
        else:
            keys = list(keys)
        keys.append(timeout)
        return self.execute_command('BLPOP', *keys)

    def brpop(self, keys, timeout=0):
        if timeout is None:
            timeout = 0
        if isinstance(keys, str_or_bytes):
            keys = [keys]
        else:
            keys = list(keys)
        keys.append(timeout)
        return self.execute_command('BRPOP', *keys)

    # SORTED SETS
    def zadd(self, name, *args, **kwargs):
        """
        Set any number of score, element-name pairs to the key ``name``. Pairs
        can be specified in two ways:

        As ``*args``, in the form of::

            score1, name1, score2, name2, ...

        or as ``**kwargs``, in the form of::

            name1=score1, name2=score2, ...

        The following example would add four values to the 'my-key' key::

            client.zadd('my-key', 1.1, 'name1', 2.2, 'name2',
                        name3=3.3, name4=4.4)
        """
        pieces = []
        if args:
            if len(args) % 2 != 0:
                raise ValueError("ZADD requires an equal number of "
                                 "values and scores")
            pieces.extend(args)
        for pair in iteritems(kwargs):
            pieces.append(pair[1])
            pieces.append(pair[0])
        return self.execute_command('ZADD', name, *pieces)

    def zinterstore(self, des, keys, weights=None, aggregate=None):
        numkeys = len(keys)
        pieces = list(keys)
        if weights:
            pieces.append(b'WEIGHTS')
            pieces.extend(weights)
        if aggregate:
            pieces.append(b'AGGREGATE')
            pieces.append(aggregate)
        return self.execute_command('ZINTERSTORE', des, numkeys, *pieces)

    def zunionstore(self, des, keys, weights=None, aggregate=None):
        numkeys = len(keys)
        pieces = list(keys)
        if weights:
            pieces.append(b'WEIGHTS')
            pieces.extend(weights)
        if aggregate:
            pieces.append(b'AGGREGATE')
            pieces.append(aggregate)
        return self.execute_command('ZUNIONSTORE', des, numkeys, *pieces)

    def zrange(self, key, start, stop, withscores=False):
        if withscores:
            return self.execute_command('ZRANGE', key, start, stop,
                                        b'WITHSCORES', withscores=True)
        else:
            return self.execute_command('ZRANGE', key, start, stop)

    def zrangebyscore(self, key, min, max, withscores=False, offset=None,
                      count=None):
        pieces = []
        if withscores:
            pieces.append(b'WITHSCORES')
        if offset:
            pieces.append(b'LIMIT')
            pieces.append(offset)
            pieces.append(count)
        return self.execute_command('ZRANGEBYSCORE', key, min, max, *pieces,
                                    withscores=withscores)

    def zrevrange(self, key, start, stop, withscores=False):
        if withscores:
            return self.execute_command('ZREVRANGE', key, start, stop,
                                        'WITHSCORES', withscores=True)
        else:
            return self.execute_command('ZRANGE', key, start, stop)

    def zrevrangebyscore(self, key, min, max, withscores=False, offset=None,
                         count=None):
        pieces = []
        if withscores:
            pieces.append(b'WITHSCORES')
        if offset:
            pieces.append(b'LIMIT')
            pieces.append(offset)
            pieces.append(count)
        return self.execute_command('ZREVRANGEBYSCORE', key, min, max, *pieces,
                                    withscores=withscores)

    def eval(self, script, keys=None, args=None):
        return self._eval('eval', script, keys, args)

    def evalsha(self, sha, keys=None, args=None):
        return self._eval('evalsha', sha, keys, args)

    def sort(self, key, start=None, num=None, by=None, get=None,
             desc=False, alpha=False, store=None, groups=False):
        '''Sort and return the list, set or sorted set at ``key``.

        ``start`` and ``num`` allow for paging through the sorted data

        ``by`` allows using an external key to weight and sort the items.
            Use an "*" to indicate where in the key the item value is located

        ``get`` allows for returning items from external keys rather than the
            sorted data itself.  Use an "*" to indicate where int he key
            the item value is located

        ``desc`` allows for reversing the sort

        ``alpha`` allows for sorting lexicographically rather than numerically

        ``store`` allows for storing the result of the sort into
            the key ``store``

        ``groups`` if set to True and if ``get`` contains at least two
            elements, sort will return a list of tuples, each containing the
            values fetched from the arguments to ``get``.

        '''
        if ((start is not None and num is None) or
                (num is not None and start is None)):
            raise CommandError("``start`` and ``num`` must both be specified")

        pieces = [key]
        if by is not None:
            pieces.append('BY')
            pieces.append(by)
        if start is not None and num is not None:
            pieces.append('LIMIT')
            pieces.append(start)
            pieces.append(num)
        if get is not None:
            # If get is a string assume we want to get a single value.
            # Otherwise assume it's an interable and we want to get multiple
            # values. We can't just iterate blindly because strings are
            # iterable.
            if isinstance(get, str):
                pieces.append('GET')
                pieces.append(get)
            else:
                for g in get:
                    pieces.append('GET')
                    pieces.append(g)
        if desc:
            pieces.append('DESC')
        if alpha:
            pieces.append('ALPHA')
        if store is not None:
            pieces.append('STORE')
            pieces.append(store)

        if groups:
            if not get or isinstance(get, str) or len(get) < 2:
                raise CommandError('when using "groups" the "get" argument '
                                   'must be specified and contain at least '
                                   'two keys')

        options = {'groups': len(get) if groups else None}
        return self.execute_command('SORT', *pieces, **options)

    def __getattr__(self, name):
        command = INVERSE_COMMANDS_INFO.get(name)
        if command:
            return Executor(self, command)
        else:
            raise AttributeError("'%s' object has no attribute '%s'" %
                                 (type(self), name))

    def _eval(self, command, script, keys, args):
        all = keys if keys is not None else ()
        num_keys = len(all)
        if args:
            all = tuple(chain(all, args))
        return self.execute(command, script, num_keys, *all)


class Pipeline(RedisClient):
    '''A :class:`.RedisClient` for pipelining commands
    '''
    def __init__(self, store):
        self.store = store
        self.reset()

    def execute(self, *args, **kwargs):
        self.command_stack.append((args, kwargs))
    execute_command = execute

    def reset(self):
        self.command_stack = []

    def commit(self, raise_on_error=True):
        '''Send commands to redis.
        '''
        cmds = list(chain([(('multi',), {})],
                          self.command_stack, [(('exec',), {})]))
        self.reset()
        return self.store.execute_pipeline(cmds, raise_on_error)


class RedisScriptMeta(type):

    def __new__(cls, name, bases, attrs):
        super_new = super(RedisScriptMeta, cls).__new__
        abstract = attrs.pop('abstract', False)
        new_class = super_new(cls, name, bases, attrs)
        if not abstract:
            o = new_class(new_class.script, new_class.__name__)
            new_class._scripts[o.name] = o
        return new_class


class RedisScript(RedisScriptMeta('_RS', (object,), {'abstract': True})):
    '''Class which helps the sending and receiving lua scripts.

    It uses the ``evalsha`` command.

    .. attribute:: script

        The lua script to run

    .. attribute:: required_scripts

        A list/tuple of other :class:`RedisScript` names required by this
        script to properly execute.

    .. attribute:: sha1

        The SHA-1_ hexadecimal representation of :attr:`script` required by the
        ``EVALSHA`` redis command. This attribute is evaluated by the library,
        it is not set by the user.

    .. _SHA-1: http://en.wikipedia.org/wiki/SHA-1
    '''
    abstract = True
    script = None
    _scripts = {}
    required_scripts = ()

    def __init__(self, script, name):
        if isinstance(script, (list, tuple)):
            script = '\n'.join(script)
        self.__name = name
        self.script = script
        rs = set((name,))
        rs.update(self.required_scripts)
        self.required_scripts = rs

    @property
    def name(self):
        return self.__name

    @property
    def sha1(self):
        if not hasattr(self, '_sha1'):
            self._sha1 = sha1(self.script.encode('utf-8')).hexdigest()
        return self._sha1

    def __repr__(self):
        return self.name if self.name else self.__class__.__name__
    __str__ = __repr__

    def preprocess_args(self, client, args):
        return args

    def callback(self, response, **options):
        '''Called back after script execution.

        This is the only method user should override when writing a new
        :class:`RedisScript`. By default it returns ``response``.

        :parameter response: the response obtained from the script execution.
        :parameter options: Additional options for the callback.
        '''
        return response

    def __call__(self, client, keys, args, options):
        args = self.preprocess_args(client, args)
        numkeys = len(keys)
        keys_args = tuple(keys) + args
        options.update({'script': self, 'redis_client': client})
        return client.execute_command('EVALSHA', self.sha1, numkeys,
                                      *keys_args, **options)
