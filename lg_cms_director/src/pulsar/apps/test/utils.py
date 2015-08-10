'''
run on arbiter
~~~~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: run_on_arbiter


sequential
~~~~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: sequential


ActorTestMixin
~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: ActorTestMixin
   :members:
   :member-order: bysource


AsyncAssert
~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: AsyncAssert
   :members:
   :member-order: bysource


check server
~~~~~~~~~~~~~~~~~~

.. autofunction:: check_server

'''
import gc
import logging
import unittest
from inspect import isclass
from functools import partial

try:
    from unittest.case import _ExpectedFailure as ExpectedFailure
except ImportError:
    ExpectedFailure = None

import pulsar
from pulsar import (get_actor, send, multi_async, async, future_timeout,
                    TcpServer, coroutine_return, new_event_loop, task,
                    format_traceback, ImproperlyConfigured, Future,
                    yield_from, iscoroutine, chain_future)
from pulsar.async.proxy import ActorProxyFuture
from pulsar.utils.importer import module_attribute
from pulsar.apps.data import create_store


__all__ = ['run_on_arbiter',
           'sequential',
           'NOT_TEST_METHODS',
           'ActorTestMixin',
           'AsyncAssert',
           'show_leaks',
           'hide_leaks',
           'check_server',
           'dont_run_with_thread',
           'run_on_actor']


LOGGER = logging.getLogger('pulsar.test')
NOT_TEST_METHODS = ('setUp', 'tearDown', '_pre_setup', '_post_teardown',
                    'setUpClass', 'tearDownClass')


class TestFailure:

    def __init__(self, exc):
        self.exc = exc
        self.trace = format_traceback(exc)

    def __str__(self):
        return '\n'.join(self.trace)


class TestCallable(object):
    '''Responsible for actually running a test function.
    '''
    def __init__(self, test, method_name, istest, timeout):
        self.test = test
        self.method_name = method_name
        self.istest = istest
        self.timeout = timeout

    def __repr__(self):
        if isclass(self.test):
            return '%s.%s' % (self.test.__name__, self.method_name)
        else:
            return '%s.%s' % (self.test.__class__.__name__, self.method_name)
    __str__ = __repr__

    def __call__(self, actor):
        __skip_traceback__ = True
        test = self.test
        if self.istest:
            test = actor.app.runner.before_test_function_run(test)
        inject_async_assert(self.test)
        test_function = getattr(test, self.method_name)
        try:
            result = test_function()
        except Exception as exc:
            result = TestFailure(exc)
        else:
            # Wraps a generator with yield_form
            timeout = self.timeout
            if iscoroutine(result):
                result = yield_from(result, timeout)
                timeout = None
            try:
                result = future_timeout(async(result), timeout)
                result.add_done_callback(partial(self._finish, actor))
                return chain_future(result, callback=self._none)
            except TypeError:
                result = None
        self._finish(actor)
        return result

    def _finish(self, actor, fut=None):
        # Callback
        if self.istest:
            actor.app.runner.after_test_function_run(self.test)

    def _none(self, _):
        pass


class SafeTest(object):
    '''Make sure the test object or class is picklable
    '''
    def __init__(self, test):
        self.test = test

    def __getattr__(self, name):
        return getattr(self.test, name)

    def __getstate__(self):
        if isclass(self.test):
            cls = self.test
            data = None
        else:
            cls = self.test.__class__
            data = self.test.__dict__.copy()
            data.pop('_loop', None)
        return ('%s.%s' % (cls.__module__, cls.__name__), data)

    def __setstate__(self, state):
        mod, data = state
        test = module_attribute(mod)
        inject_async_assert(test)
        if data is not None:
            test = test.__new__(test)
            test.__dict__.update(data)
        self.test = test


class TestFunction(object):

    def __init__(self, method_name):
        self.method_name = method_name
        self.istest = self.method_name not in NOT_TEST_METHODS

    def __repr__(self):
        return self.method_name
    __str__ = __repr__

    def __call__(self, test, timeout):
        callable = TestCallable(test, self.method_name, self.istest, timeout)
        return callable(get_actor())


class TestFunctionOnArbiter(TestFunction):

    def __call__(self, test, timeout):
        test = SafeTest(test)
        callable = TestCallable(test, self.method_name, self.istest, timeout)
        actor = get_actor()
        if actor.is_monitor():
            return callable(actor)
        else:
            # send the callable to the actor monitor
            return actor.send(actor.monitor, 'run', callable)


def run_on_arbiter(f):
    '''Decorator for running a test function in the :class:`.Arbiter`
    context domain.

    This can be useful to test Arbiter mechanics.
    '''
    f.testfunction = TestFunctionOnArbiter(f.__name__)
    return f


def run_on_actor(cls):
    '''Decorator for a :class:`~unittest.TestCase` which cause
    its test functions to run on the actor evnet loop rather than
    in the executor
    '''
    cls._actor_execution = True
    return cls


def sequential(cls):
    '''Decorator for a :class:`~unittest.TestCase` which cause
    its test functions to run sequentially rather than in an
    asynchronous fashion.

    Typical usage::

        import unittest

        from pulsar.apps.test import sequential

        @sequenatial
        class MyTests(unittest.TestCase):
            ...

    You can also run test functions sequentially when using the
    :ref:`sequential <apps-test-sequential>` flag in the command line.
    '''
    cls._sequential_execution = True
    return cls


class AsyncAssert(object):
    '''A `descriptor`_ added by the :ref:`test-suite` to all python
    :class:`~unittest.TestCase` loaded.

    It can be used to invoke the same ``assertXXX`` methods available in
    the :class:`~unittest.TestCase` in an asynchronous fashion.

    The descriptor is available via the ``async`` attribute.
    For example::

        class MyTest(unittest.TestCase):

            def test1(self):
                yield self.async.assertEqual(3, Future().callback(3))
                ...


    .. _descriptor: http://users.rcn.com/python/download/Descriptor.htm
    '''
    def __init__(self, test):
        self.test = test

    def __get__(self, instance, instance_type=None):
        if instance is not None:
            return AsyncAssert(instance)
        else:
            return self

    def __getattr__(self, name):

        @task
        def _(*args, **kwargs):
            __skip_traceback__ = True
            args = yield multi_async(args)
            result = yield getattr(self.test, name)(*args, **kwargs)
            coroutine_return(result)

        return _

    @task
    def assertRaises(self, error, callable, *args, **kwargs):
        try:
            yield callable(*args, **kwargs)
        except error:
            coroutine_return()
        except Exception:
            raise self.test.failureException('%s not raised by %s'
                                             % (error, callable))
        else:
            raise self.test.failureException('%s not raised by %s'
                                             % (error, callable))


class ActorTestMixin(object):
    '''A mixin for :class:`~unittest.TestCase`.

    Useful for classes testing spawning of actors.
    Make sure this is the first class you derive from, before the
    :class:`~unittest.TestCase`, so that the tearDown method is overwritten.

    .. attribute:: concurrency

        The concurrency model used to spawn actors via the :meth:`spawn`
        method.
    '''
    concurrency = 'thread'

    @property
    def all_spawned(self):
        if not hasattr(self, '_spawned'):
            self._spawned = []
        return self._spawned

    @task
    def spawn_actor(self, concurrency=None, **kwargs):
        '''Spawn a new actor and perform some tests
        '''
        concurrency = concurrency or self.concurrency
        ad = pulsar.spawn(concurrency=concurrency, **kwargs)
        self.assertTrue(ad.aid)
        self.assertTrue(isinstance(ad, ActorProxyFuture))
        proxy = yield ad
        self.all_spawned.append(proxy)
        self.assertEqual(proxy.aid, ad.aid)
        self.assertEqual(proxy.proxy, proxy)
        self.assertTrue(proxy.cfg)
        coroutine_return(proxy)

    def stop_actors(self, *args):
        all = args or self.all_spawned
        return multi_async((send(a, 'stop') for a in all))

    def tearDown(self):
        return self.stop_actors()


def inject_async_assert(obj):
    tcls = obj if isclass(obj) else obj.__class__
    if not hasattr(tcls, 'async'):
        tcls.async = AsyncAssert(tcls)


def show_leaks(actor, show=True):
    '''Function to show memory leaks on a processed-based actor.'''
    if not actor.is_process():
        return
    gc.collect()
    if gc.garbage:
        MAX_SHOW = 100
        write = actor.stream.writeln if show else lambda msg: None
        write('MEMORY LEAKS REPORT IN %s' % actor)
        write('Created %s uncollectable objects' % len(gc.garbage))
        for obj in gc.garbage[:MAX_SHOW]:
            write('Type: %s' % type(obj))
            write('=================================================')
            write('%s' % obj)
            write('-------------------------------------------------')
            write('')
            write('')
        if len(gc.garbage) > MAX_SHOW:
            write('And %d more' % (len(gc.garbage) - MAX_SHOW))


def hide_leaks(actor):
    show_leaks(actor, False)


def check_server(name):
    '''Check if server ``name`` is available at the address specified
    ``<name>_server`` config value.

    :rtype: boolean
    '''
    cfg = get_actor().cfg
    addr = cfg.get('%s_server' % name)
    if addr:
        if ('%s://' % name) not in addr:
            addr = '%s://%s' % (name, addr)
        try:
            sync_store = create_store(addr, loop=new_event_loop())
        except ImproperlyConfigured:
            return False
        try:
            sync_store.ping()
            return True
        except Exception:
            return False
    else:
        return False


def dont_run_with_thread(obj):
    '''Decorator for disabling process based test cases when the test suite
    runs in threading, rather than processing, mode.
    '''
    actor = pulsar.get_actor()
    if actor:
        d = unittest.skipUnless(actor.cfg.concurrency == 'process',
                                'Run only when concurrency is process')
        return d(obj)
    else:
        return obj


def is_expected_failure(exc, default=False):
    if ExpectedFailure:
        return isinstance(exc, ExpectedFailure)
    else:
        return default
