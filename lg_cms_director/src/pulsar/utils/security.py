"""\
Security related helpers such as secure password hashing tools.
"""
from hashlib import sha1
from uuid import uuid4
import string
from random import SystemRandom, choice

from .pep import range
from .httpurl import ascii_letters

SALT_CHARS = ascii_letters + string.digits


_sys_rng = SystemRandom()


def gen_unique_id():
    return 'i%s' % uuid4().hex


def gen_salt(length):
    """Generate a random string of SALT_CHARS with specified ``length``."""
    if length <= 0:
        raise ValueError('requested salt of length <= 0')
    return ''.join(_sys_rng.choice(SALT_CHARS) for _ in range(length))


def _hash_internal(salt, password):
    return sha1(('%s%s' % (salt, password)).encode('utf-8')).hexdigest()


def generate_password_hash(password, salt_length=8):
    salt = gen_salt(salt_length)
    h = _hash_internal(salt, password)
    return '%s$%s' % (salt, h)


def check_password_hash(pwhash, password):
    if pwhash.count('$') != 1:
        return False
    salt, hashval = pwhash.split('$')
    return _hash_internal(salt, password) == hashval


def random_string(characters=None, length=None):
    length = length or 20
    characters = characters or SALT_CHARS
    return ''.join((choice(characters) for s in range(length)))
