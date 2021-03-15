import bind_nested_structures

b = bind_nested_structures.values()  # <-- don't forget the parenthesis !
b.names.a = 30
print(b.names.a)  # prints 30
b.names.d[0] = 3
print(b.names.d[0])
