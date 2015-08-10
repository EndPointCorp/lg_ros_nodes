'''
Tha main component for pulsar datastore clients is the :class:`.Store`
class which encapsulates the essential API for communicating and executing
commands on remote servers.
A :class:`.Store` can also implement several methods for managing
the higher level :ref:`object data mapper <odm>`.
'''
from pulsar import ImproperlyConfigured, Pool, Producer
from pulsar.utils.importer import module_attribute
from pulsar.utils.pep import to_string
from pulsar.utils.httpurl import urlsplit, parse_qsl, urlunparse, urlencode

__all__ = ['Command',
           'Store',
           'PubSub',
           'PubSubClient',
           'parse_store_url',
           'create_store',
           'register_store',
           'data_stores']

data_stores = {}


REV_KEY = '_rev'


class Command(object):
    '''A command executed during a in a :meth:`~.Store.execute_transaction`

    .. attribute:: action

        Type of action:
        * 0 custom command
        * 1 equivalent to an SQL INSERT
        * 2 equivalent to an SQL DELETE
    '''
    __slots__ = ('args', 'action')
    INSERT = 1
    UPDATE = 2
    DELETE = 3

    def __init__(self, args, action=0):
        self.args = args
        self.action = action

    @classmethod
    def insert(cls, args):
        return cls(args, cls.INSERT)


class StoreTransaction(object):
    '''Transaction for a given :class:`.Store`
    '''
    def __init__(self, store):
        self.store = store
        self.commands = []

    def add(self, model):
        action = Command.UPDATE if REV_KEY in model else Command.INSERT
        self.commands.append(Command(model, action))
        return model


class Compiler(object):
    '''Interface for :class:`Store` compilers.
    '''
    def __init__(self, store):
        self.store = store

    def compile_query(self, query):
        raise NotImplementedError

    def create_table(self, model_class):
        raise NotImplementedError


class Store(Producer):
    '''Base class for an asynchronous :ref:`data stores <data-stores>`.

    It is an :class:`.Producer` for accessing and retrieving
    data from remote data servers such as redis, couchdb and so forth.
    A :class:`Store` should not be created directly, the high level
    :func:`.create_store` function should be used instead.

    .. attribute:: _host

        The remote host, tuple or string

    .. attribute:: _user

        The user name

    .. attribute:: _password

        The user password
    '''
    _scheme = None
    compiler_class = None
    default_manager = None
    registered = False
    MANY_TIMES_EVENTS = ('request',)

    def __init__(self, name, host, loop=None, database=None,
                 user=None, password=None, encoding=None, **kw):
        super(Store, self).__init__(loop)
        self._name = name
        self._host = host
        self._encoding = encoding or 'utf-8'
        self._database = database
        self._user = user
        self._password = password
        self._urlparams = {}
        self._init(**kw)
        self._dns = self._buildurl()

    @property
    def name(self):
        '''Store name'''
        return self._name

    @property
    def database(self):
        '''Database name/number associated with this store.'''
        return self._database

    @property
    def encoding(self):
        '''Store encoding (usually ``utf-8``)
        '''
        return self._encoding

    @property
    def dns(self):
        '''Domain name server'''
        return self._dns

    @classmethod
    def register(cls):
        pass

    def __repr__(self):
        return 'Store(dns="%s")' % self._dns
    __str__ = __repr__

    def connect(self):
        '''Connect with store server
        '''
        raise NotImplementedError

    def execute(self, *args, **options):
        '''Execute a command
        '''
        raise NotImplementedError

    def ping(self):
        '''Used to check if the data server is available
        '''
        raise NotImplementedError

    def client(self):
        '''Get a client for the Store if implemented
        '''
        raise NotImplementedError

    def pubsub(self, **kw):
        '''Obtain a :class:`PubSub` handler for the Store if implemented
        '''
        raise NotImplementedError

    def create_database(self, dbname=None, **kw):
        '''Create a new database in this store.

        By default it does nothing, stores must implement this method
        only if they support database creation.

        :param dbname: optional database name. If not supplied a
            database with :attr:`database` is created.
        '''
        pass

    def delete_database(self, dbname=None):
        '''Delete a database ``dbname``

        By default it does nothing, stores must implement this method
        only if they support database deletion.

        :param dbname: optional database name. If not supplied a
            database named :attr:`database` is deleted.
        '''
        pass

    def close(self):
        '''Close all open connections
        '''
        raise NotImplementedError

    def flush(self):
        '''Flush the store.'''
        raise NotImplementedError

    # encode/decode field values
    def encode_bytes(self, data):
        '''Encode bytes ``data``

        :param data: a bytes string
        :return: bytes or string
        '''
        return data

    def dencode_bytes(self, data):
        '''Decode bytes ``data``

        :param data: bytes or string
        :return: bytes
        '''
        return data

    def encode_bool(self, data):
        return bool(data)

    def encode_json(self, data):
        return data

    #    ODM SUPPORT
    #######################
    def transaction(self):
        '''Create a transaction for this store.
        '''
        return StoreTransaction(self)

    def create_table(self, model, remove_existing=False):
        '''Create the table for ``model``.

        This method is used by the :ref:`object data mapper <odm>`.
        By default it does nothing.
        '''

    def drop_table(self, model, remove_existing=False):
        '''Drop the table for ``model``.

        This method is used by the :ref:`object data mapper <odm>`.
        By default it does nothing.
        '''

    def table_info(self, model):
        '''Information about the table/collection mapping ``model``
        '''
        pass

    def create_model(self, manager, *args, **kwargs):
        '''Create a new model from a ``manager``

        Method used by the :class:`.Manager` callable method.
        '''
        instance = manager._model(*args, **kwargs)
        instance['_mapper'] = manager._mapper
        return instance

    def execute_transaction(self, transaction):
        '''Execute a  :meth:`transaction` in a multi-store
        :class:`.Transaction`.

        THis methid is used by the :ref:`object data mapper <odm>` and
        should not be invoked directly.
        It returns a list of models committed to the backend server.
        '''
        raise NotImplementedError

    def compile_query(self, query):
        '''Compile the :class:`.Query` ``query``.

        Method required by the :class:`Object data mapper <odm>`.

        :return: an instance of :class:`.CompiledQuery` if implemented
        '''
        raise NotImplementedError

    def get_model(self, manager, pkvalue):
        '''Fetch an instance of a ``model`` with primary key ``pkvalue``.

        This method required by the :ref:`object data mapper <odm>`.

        :param manager: the :class:`.Manager` calling this method
        :param pkvalue: the primary key of the model to retrieve
        '''
        raise NotImplementedError

    def has_query(self, query_type):
        '''Check if this :class:`.Store` supports ``query_type``.

        :param query_type: a string indicating the query type to check
            (``filter``, ``exclude``, ``search``).

        This method is used by the :ref:`object data mapper <odm>`.
        '''
        return True

    def build_model(self, manager, *args, **kwargs):
        instance = manager(*args, **kwargs)
        instance['_store'] = self
        return instance

    def model_data(self, model, action):
        '''Generator of ``field, value`` pair for the data store.

        By default invokes the :class:`.ModelMeta.store_data` method.'''
        return model._meta.store_data(model, self, action)

    #    INTERNALS
    #######################
    def _init(self, **kw):  # pragma    nocover
        '''Internal initialisation'''
        pass

    def _buildurl(self, **kw):
        pre = ''
        if self._user:
            if self._password:
                pre = '%s:%s@' % (self._user, self._password)
            else:
                pre = '%s@' % self._user
        elif self._password:
            raise ImproperlyConfigured('password but not user')
            assert self._password
        host = self._host
        if isinstance(host, tuple):
            host = '%s:%s' % host
        host = '%s%s' % (pre, host)
        path = '/%s' % self._database if self._database else ''
        kw.update(self._urlparams)
        query = urlencode(kw)
        scheme = self._name
        if self._scheme:
            scheme = '%s+%s' % (self._scheme, scheme)
        return urlunparse((scheme, host, path, '', query, ''))

    def _build_pool(self):
        return Pool


class PubSubClient(object):
    '''Interface for a client of :class:`PubSub` handler.

    Instances of this :class:`Client` are callable object and are
    called once a new message has arrived from a subscribed channel.
    The callable accepts two parameters:

    * ``channel`` the channel which originated the message
    * ``message`` the message
    '''
    def __call__(self, channel, message):
        raise NotImplementedError


class PubSub(object):
    '''A Publish/Subscriber interface.

    A :class:`PubSub` handler is never initialised directly, instead,
    the :meth:`~Store.pubsub` method of a data :class:`.Store`
    is used.

    To listen for messages one adds clients to the handler::

        def do_somethind(channel, message):
            ...

        pubsub = client.pubsub()
        pubsub.add_client(do_somethind)
        pubsub.subscribe('mychannel')

    You can add as many listening clients as you like. Clients are functions
    which receive two parameters only, the ``channel`` sending the message
    and the ``message``.

    A :class:`PubSub` handler can be used to publish messages too::

        pubsub.publish('mychannel', 'Hello')

    An additional ``protocol`` object can be supplied. The protocol must
    implement the ``encode`` and ``decode`` methods.
    '''

    def __init__(self, store, protocol=None):
        super(PubSub, self).__init__()
        self.store = store
        self._loop = store._loop
        self._protocol = protocol
        self._connection = None
        self._clients = set()

    def publish(self, channel, message):
        '''Publish a new ``message`` to a ``channel``.
        '''
        raise NotImplementedError

    def count(self, *channels):
        '''Returns the number of subscribers (not counting clients
        subscribed to patterns) for the specified channels.
        '''
        raise NotImplementedError

    def channels(self, pattern=None):
        '''Lists the currently active channels.

        An active channel is a Pub/Sub channel with one ore more subscribers
        (not including clients subscribed to patterns).
        If no ``pattern`` is specified, all the channels are listed,
        otherwise if ``pattern`` is specified only channels matching the
        specified glob-style pattern are listed.
        '''
        raise NotImplementedError

    def psubscribe(self, pattern, *patterns):
        '''Subscribe to a list of ``patterns``.
        '''
        raise NotImplementedError

    def punsubscribe(self, *channels):
        '''Unsubscribe from a list of ``patterns``.
        '''
        raise NotImplementedError

    def subscribe(self, channel, *channels):
        '''Subscribe to a list of ``channels``.
        '''
        raise NotImplementedError

    def unsubscribe(self, *channels):
        '''Un-subscribe from a list of ``channels``.
        '''
        raise NotImplementedError

    def close(self):
        '''Stop listening for messages.
        '''
        raise NotImplementedError

    def add_client(self, client):
        '''Add a new ``client`` to the set of all :attr:`clients`.

        Clients must be callable accepting two parameters, the channel and
        the message. When a new message is received
        from the publisher, the :meth:`broadcast` method will notify all
        :attr:`clients` via the ``callable`` method.'''
        self._clients.add(client)

    def remove_client(self, client):
        '''Remove *client* from the set of all :attr:`clients`.'''
        self._clients.discard(client)

    # INTERNALS
    def broadcast(self, response):
        '''Broadcast ``message`` to all :attr:`clients`.'''
        remove = set()
        channel = to_string(response[0])
        message = response[1]
        if self._protocol:
            message = self._protocol.decode(message)
        for client in self._clients:
            try:
                client(channel, message)
            except IOError:
                remove.add(client)
            except Exception:
                self._loop.logger.exception(
                    'Exception while processing pub/sub client. Removing it.')
                remove.add(client)
        self._clients.difference_update(remove)


def parse_store_url(url):
    assert url, 'No url given'
    scheme, host, path, query, fr = urlsplit(url)
    assert not fr, 'store url must not have fragment, found %s' % fr
    assert scheme, 'Scheme not provided'
    # pulsar://
    if scheme == 'pulsar' and not host:
        host = '127.0.0.1:0'
    bits = host.split('@')
    assert len(bits) <= 2, 'Too many @ in %s' % url
    params = dict(parse_qsl(query))
    if path:
        database = path[1:]
        assert '/' not in database, 'Unsupported database %s' % database
        params['database'] = database
    if len(bits) == 2:
        userpass, host = bits
        userpass = userpass.split(':')
        assert len(userpass) <= 2,\
            'User and password not in user:password format'
        params['user'] = userpass[0]
        if len(userpass) == 2:
            params['password'] = userpass[1]
    if ':' in host:
        host = tuple(host.split(':'))
        host = host[0], int(host[1])
    return scheme, host, params


def create_store(url, **kw):
    '''Create a new :class:`Store` for a valid ``url``.

    :param url: a valid ``url`` takes the following forms:

        :ref:`Pulsar datastore <store_pulsar>`::

            pulsar://user:password@127.0.0.1:6410

        :ref:`Redis <store_redis>`::

            redis://user:password@127.0.0.1:6500/11?namespace=testdb

        :ref:`CouchDb <store_couchdb>`::

            couchdb://user:password@127.0.0.1:6500/testdb
            https+couchdb://user:password@127.0.0.1:6500/testdb

    :param kw: additional key-valued parameters to pass to the :class:`.Store`
        initialisation method. It can contains parameters such as
        ``database``, ``user`` and ``password`` to override the
        ``url`` values. Additional parameters are processed by the
        :meth:`.Store._init` method.
    :return: a :class:`Store`.
    '''
    if isinstance(url, Store):
        return url
    scheme, address, params = parse_store_url(url)
    dotted_path = data_stores.get(scheme)
    if not dotted_path:
        raise ImproperlyConfigured('%s store not available' % scheme)
    store_class = module_attribute(dotted_path)
    if not store_class.registered:
        store_class.registered = True
        store_class.register()
    params.update(kw)
    return store_class(scheme, address, **params)


def register_store(name, dotted_path):
    '''Register a new :class:`.Store` with schema ``name`` which
    can be found at the python ``dotted_path``.
    '''
    data_stores[name] = dotted_path
