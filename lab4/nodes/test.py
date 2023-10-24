class Test(object):
    def __init__(self) -> None:
        self.x = 10
        self.y = 20
        pass
    def add(self):
        print(self.x+self.y)
        pass

test = Test()
test.add()
