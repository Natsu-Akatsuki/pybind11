import throw_exception

if __name__ == '__main__':
    try:
        throw_exception.throw_exception()
    except Exception as e:
        print(repr(e))