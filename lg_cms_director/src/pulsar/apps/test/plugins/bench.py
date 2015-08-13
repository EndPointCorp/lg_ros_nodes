'''
:class:`.BenchMark` is a :class:`.TestPlugin` for benchmarking test functions.

To use the plugin follow these three steps:

* Included it in the test Suite::

    from pulsar.apps.test import TestSuite
    from pulsar.apps.test.plugins import bench

    def suite():
        TestSuite(..., plugins=(..., bench.BenchMark()))

* Flag a ``unittest.TestCase`` class with the ``__benchmark__ = True``
  class attribute::

      class MyBenchmark(unittest.TestCase):
          __benchmark__ = True

          def test_mybenchmark_function1(self):
              ...

          def test_mybenchmark_function2(self):
              ...

* Run the test suite with the ``--benchmark`` command line option.

The test class can implement additional methods to fine-tune how the
benchmark plugin evaluate the perfomance and display results:

* When implemented, the ``startUp`` method is invoked before each run
  of a test function.
* The time taken to run a test once can be modified by implementing
  the ``getTime`` method which receives as only argument the time interval
  taken.
  By default it returns the same time interval.

.. autoclass:: BenchMark

'''
import sys
import time
import math

if sys.platform == "win32":  # pragma    nocover
    default_timer = time.clock
else:
    default_timer = time.time

from unittest import TestSuite

import pulsar
from pulsar.utils.pep import range

from pulsar.apps import test


BENCHMARK_TEMPLATE = ('{0[name]}: repeated {0[number]} times, '
                      'average {0[mean]} secs, stdev {0[std]}')


class BenchTest(test.WrapTest):

    def __init__(self, test, number):
        super(BenchTest, self).__init__(test)
        self.number = number

    def updateSummary(self, info, number, total_time, total_time2):
        mean = total_time/number
        std = math.sqrt((total_time2 - total_time*mean)/number)
        std = round(100*std/mean, 2)
        info.update({'number': number,
                     'mean': '%.5f' % mean,
                     'std': '{0} %'.format(std)})

    def _call(self):
        simple = lambda info, *args: info
        testMethod = self.testMethod
        testStartUp = getattr(self.test, 'startUp', lambda: None)
        testGetTime = getattr(self.test, 'getTime', lambda dt: dt)
        testGetInfo = getattr(self.test, 'getInfo', simple)
        testGetSummary = getattr(self.test, 'getSummary', simple)
        t = 0
        t2 = 0
        info = {'name': '%s.%s' % (self.test.__class__.__name__,
                                   testMethod.__name__)}
        for r in range(self.number):
            testStartUp()
            start = default_timer()
            testMethod()
            delta = default_timer() - start
            dt = testGetTime(delta)
            testGetInfo(info, delta, dt)
            t += dt
            t2 += dt*dt
        self.updateSummary(info, self.number, t, t2)
        self.set_test_attribute('bench_info',
                                testGetSummary(info, self.number, t, t2))


class BenchMark(test.TestPlugin):
    '''Benchmarking addon for pulsar test suite.'''
    desc = '''Run benchmarks function flagged with __benchmark__ attribute'''

    repeat = pulsar.Setting(flags=['--repeat'],
                            type=int,
                            default=1,
                            validator=pulsar.validate_pos_int,
                            desc=('Default number of repetition '
                                  'when benchmarking.'))

    def loadTestsFromTestCase(self, test_cls):
        bench = getattr(test_cls, '__benchmark__', False)
        if self.config.benchmark != bench:  # skip the loading
            return TestSuite()

    def before_test_function_run(self, test, local):
        if self.config.benchmark:
            method_name = getattr(test, '_testMethodName', None)
            if method_name:
                method = getattr(test, method_name, None)
                bench = getattr(test, '__benchmark__', False)
                if not bench and method:
                    bench = getattr(method, '__benchmark__', False)
                if bench:
                    number = getattr(test, '__number__', self.config.repeat)
                    return BenchTest(test, number)

    def addSuccess(self, test):
        if self.config.benchmark and self.stream:
            result = getattr(test, 'bench_info', None)
            # if result and self.stream.showAll:
            if result:
                stream = self.stream.handler('benchmark')
                template = getattr(test, 'benchmark_template',
                                   BENCHMARK_TEMPLATE)
                stream.writeln(template.format(result))
                stream.flush()
                self.result.addSuccess(test)
                return True

    def addError(self, test, err):
        msg = self._msg(test, 'ERROR')
        if msg:
            self.result.addError(test, err)
            return msg

    def addFailure(self, test, err):
        msg = self._msg(test, 'FAILURE')
        if msg:
            self.result.addFailure(test, err)
            return msg

    def addSkip(self, test, reason):
        msg = self._msg(test, 'SKIPPED')
        if msg:
            self.result.addSkip(test, reason)
            return msg

    def _msg(self, test, msg):
        if self.config.benchmark and self.stream:
            stream = self.stream.handler('benchmark')
            stream.writeln('%s: %s' % (test, msg))
            stream.flush()
            return True
