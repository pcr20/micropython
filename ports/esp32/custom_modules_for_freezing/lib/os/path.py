import os
import uos

sep = "/"


def normcase(s):
    return s


def normpath(s):
    return s


def abspath(s):
    if s[0] != "/":
        return os.getcwd() + "/" + s
    return s


def join(*args):
    # TODO: this is non-compliant
    if type(args[0]) is bytes:
        return b"/".join(args)
    else:
        return "/".join(args)


def split(path):
    if path == "":
        return ("", "")
    r = path.rsplit("/", 1)
    if len(r) == 1:
        return ("", path)
    head = r[0]  # .rstrip("/")
    if not head:
        head = "/"
    return (head, r[1])


def dirname(path):
    return split(path)[0]


def basename(path):
    return split(path)[1]


def exists(path):
    return os.access(path, os.F_OK)


# TODO
lexists = exists

#Python 3.7.4 (default, Aug 13 2019, 15:17:50)
#[Clang 4.0.1 (tags/RELEASE_401/final)] :: Anaconda, Inc. on darwin
#Type "help", "copyright", "credits" or "license" for more information.
#>>> import stat
#>>> hex(stat.S_IFREG)
#'0x8000'
#>>> hex(stat.S_IFDIR)
#'0x4000'
#>>>
def isdir(path):
    try:
        if uos.stat(path)[0] & 0x4000: #hex(stat.S_IFDIR)
            return True
        else:
            return False
    except OSError:
        return False

def isfile(path):
    try:
        if uos.stat(path)[0] & 0x8000: #hex(stat.S_IFREG) https://ironpython-test.readthedocs.io/en/latest/library/stat.html
            return True
        else:
            return False
    except OSError:
            return False

def getsize(path):
    return uos.stat(path)[6]


def expanduser(s):
    if s == "~" or s.startswith("~/"):
        h = os.getenv("HOME")
        return h + s[1:]
    if s[0] == "~":
        # Sorry folks, follow conventions
        return "/home/" + s[1:]
    return s
