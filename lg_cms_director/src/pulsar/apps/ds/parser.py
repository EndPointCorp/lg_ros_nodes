import pulsar

from .pyparser import Parser


class ResponseError(pulsar.PulsarException):
    pass


class InvalidResponse(pulsar.PulsarException):
    pass


class NoScriptError(ResponseError):
    pass


EXCEPTION_CLASSES = {
    'ERR': ResponseError,
    'NOSCRIPT': NoScriptError,
}


def response_error(response):
    "Parse an error response"
    response = response.split(' ')
    error_code = response[0]
    if error_code not in EXCEPTION_CLASSES:
        error_code = 'ERR'
    response = ' '.join(response[1:])
    return EXCEPTION_CLASSES[error_code](response)


PyRedisParser = lambda: Parser(InvalidResponse, response_error)


if pulsar.HAS_C_EXTENSIONS:
    from pulsar.utils.lib import RedisParser as _RedisParser
    RedisParser = lambda: _RedisParser(InvalidResponse, response_error)
else:    # pragma nocover
    RedisParser = PyRedisParser


def redis_parser(py_redis_parser=False):
    return PyRedisParser if py_redis_parser else RedisParser
