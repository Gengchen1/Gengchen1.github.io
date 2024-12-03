---
title: 【Cpp学习笔记】Cpp_Cherno
date: 2024-09-11
summary: Cherno是一个国外C++大佬，C++讲的很好，墙裂推荐~。
category: Cpp
tags: [Cpp]
comments: true
---

## 36. 构造函数中初始化列表

    一种初始化类成员变量的方式。

一般的初始化方式：

```cpp
#include <iostream>

class Entity
{
private:
	std::string m_Name;
public:
	Entity()
	{
		m_Name = "Unknown";
	}
	Entity(const std::string& name) // &表示name变量是引用输入的变量。
	{
		m_Name = name;
	}
	const std::string& GetName() const { return m_Name; } // 参数列表后的const表示方法是不改变成员变量。
};

int main()
{
	Entity e;
	std::cout << e.GetName() << std::endl;

	Entity e0("Cherno");
	std::cout << e0.GetName() << std::endl;
	std::cin.get();
}
```

通过初始化列表来初始化成员变量：

```cpp
class Entity
{
private:
	std::string m_Name;
	int m_Score;
public:
	Entity()
		: m_Name("Cherno"), m_Score(0)
	{
	}
	Entity(const std::string& name)
		: m_Name(name), m_Score(0)
	{
	}
};
```

    注意：使用初始化列表时，初始化的变量顺序要和定义变量时的一致。
    因为不管在初始化列表中如何写，他都会按照类成员的定义顺序进行初始化。

为什么要使用初始化列表来初始化成员变量？

> 因为如果我们有很多变量要初始化：
> private:
> int x , y , z;
> public:
> Entity()
> {
> int x = 0;
> int y = 0;
> int z = 0;
> }
> 这样显得很杂乱，构造函数中都是初始化变量了。
> 而这样写就便于阅读;
```cpp
private:
    int x , y , z;
public:
    Entity()
         : x(0), y(0), z(0) {}
```
> 第二个原因是
> 写在函数体内,会创建两个字符串, 一次默认构造函数创建,一次是 std::string("Unknown")参数,造成了资源浪费.

```cpp
class Example
{
public:
    Example()
    {
        std::cout << "Created Entity !"<< std::endl;
    }

    Example(int x)
    {
        std::cout << "Created Entity with" << x << "!" << std::endl;
    }
};

class Entity
{
private:
    std::string m_Name;
    int m_Score;
    Example m_Example; // 实例化对象时，这里创建一次。
public:
    Entity()
    {
        m_Name = "Unknown";
        m_Example = Example(8); // 赋值参数8时再次创建一次。
    }
    Entity(const std::string &name)
    {
        m_Name = name;
    }
}
结果：
Created Entity !
Created Entity with8!
```

而对于整数这种基础类型，不会被初始化，除非显式赋值初始化它们。

> Sg: 建议所有地方都用初始化列表。

## 37. 三元运算符

    由 : 和 ？组成，只是 `if` 语句的语法糖。

当需要根据一种特定的条件给变量赋值时，可以用一个问号加冒号来代替 `if else` 语句。
两个例子：

```cpp
static int s_Level = 8;
static int s_Speed = 2;
// 1.
s_Speed = s_Level > 5 ? 10 : 5;
// 2.
std::string rank = s_Level > 10 ? "Master" : "Beginner"; // 这样写不仅整洁，而且不会多创建一个临时变量。
```

## 38. 如何创建对象

在C++中可以选择对象在那里创建，堆上还是栈上。

**栈分配**

```cpp
// 在栈中创建
Entity entity;
Entity entity("lk"); // 等价于 Entity entity = Entity("lk");
```

- 分配栈的情况：几乎任何时候，因为在C++中栈分配是初始化对象最快和最受管控的方式（不需要手动释放内存）
- 不栈分配的情况：1. 如果创建的对象很大，或者很多，栈的内存有限 2. 要显式地控制对象的生存周期。就需要在堆上创建。

**堆分配**

```cpp
Entity* entity = new Entity("lk");
delete entity; //使用完对象后，要手动进行删除。
```

- 当使用new关键字时，就是在堆上分配了内存,**而且调用类的构造函数**，new 关键字会返回该内存的地址，即Entity指针，所以要用Entity\* 类型。

## 39. new 关键字

    new 关键字的作用是分配一块堆上的内存，用法是 new + 数据类型。
    根据所写的数据类型，以字节为单位决定了要分配的内存大小。

- new关键字后面加类时，做了两件事： 1. 在堆上为这个类分配了一块内存 2. 调用了这个类的构造函数。
- new关键字的底层其实就是调用了C标准库里的 `malloc()` 函数来分配内存，然后又调用了 `Entity` 的构造函数。
- delete则会调用destructor析构函数。

```cpp
Entity* e = new Entity();
Entity* e = (Entity*)malloc(sizeof(Entity));
//以上两行代码唯一的区别就是new关键字还调用了Entity的构造函数。

```

> 如果使用new和[]来分配数组，那么也要用delete[]

- new支持一种叫 placement new 的用法，这决定了他的内存来自那里，所以其实并没有真正分配内存，在这种情况下，你只需要调用构造函数，并在一个特定的内存地址中初始化你的Entity，可以通过些new()然后指定内存地址，例如：

```cpp
int* b = new int[50];
Entity* entity = new(b) Entity();
```

## 40. 隐式转换

C++允许编译器对代码进行一次隐式的转换。- 比如开始将一种数据类型作为另一种类型来使用时，在这两种类型之间就会有类型转换。

```cpp
class Entity
{
private:
	std::string m_Name;
	int m_Age;
public:
	Entity(const std::string& name)
		: m_Name(name), m_Age(-1) {}
	Entity(int age)
		: m_Name("Unknown"), m_Age(age) {}
};

int main()
{
	Entity a("Cherno"); // 相当于 Entity a = Entity("Cherno")
	Entity b(23); // 相当于 Entity b = Entity(23);
	// 而下面这种也可以：
	Entity a = "Cherno"; // error，因为“Cherno” 是一个字符数组类型，而不是字符串类型。隐式转化只能一次。
	Entity b = 23;
	// 这里就进行了隐式转换，将隐式地将23转换为了一个Entity对象,因为Entity有一个接受整形参数的构造函数。

	std::cin.get();
}
```

```cpp
Entity a = 22; // 尽量不要这样写
Entity a(22);
```

**explicit关键字**
如果使用 `explicit` 关键字，会禁用隐式转换:

```cpp
explicit Entity(int age)
	: m_Name("Unknown"), m_Age(age) {}
// explicit 关键字放在构造函数前面

Entity a = 22; // error!
```

在数学运算库的地方会用到关键字 `explicit` , 因为不想把数字和向量作比较:

> 假设你有一个`Vector`类，这个类有一个接受一个浮点数的构造函数，用于创建一个长度为该浮点数的向量, 如果不用关键字 `explicit` , 可能会不小心比较了向量和浮点数,而如果使用了 `explicit`

```cpp
class Vector {
public:
	explicit Vector(float length)
		: m_Length(length) {}
private:
	float m_Length
}
// 写比较浮点数和向量的时候就会报错
if (3.14 == Vector(3.14))
{}
```

## 41. 操作符和操作符重载

什么是操作符?

>     操作符是一种用来代替函数执行某些事情的符号, 加减, 解引用, 取址, 左移 << , 逗号, 圆括号,方括号, new, delete.

什么是重载(overloading)?

> 重载在操作符这里的意思就是给他一个新的含义,或添加参数,或重新创建.

    重载允许在程序中定义或更改一个操作符的行为.

_可以说,操作符就是一个函数_

**总的来说: 应该谨慎且少用操作符重载, 除非非常有意义 **

`*` 和 `+` 操作符的重载：

```cpp
struct Vector2
{
	float x, float y;

	Vector2 operator+(const Vector2& other) const
	{
		return Vector2(x + other.x, y + other.y)
	}
	Vector2 operator*(const Vector2& other) const // 传入一个引用的Vector, 然后函数后加const表示不改变结构体成员。
	{
		return Vector2(x * other.x, y * other.y) // 返回一个相加后的Vector.
	}
};

int main()
{
	Vector2 position(4.0f, 4.0f);
	Vector2 speed(0.5f, 1.5f);
	Vector2 speed(0.5f, 1.5f);
	Vector2 powerup(1.1f, 1.1f);

	Vector2 result = position + speed * powerup;
}

```

`<<` 操作符的重载

```cpp
std::cout << result << std::endl; // error! 因为result这变量的类型没有重载 << 操作符
std::cout << "Zhaogengchen" << std::endl; // ok! 因为 `"Zhaogengchen"` 是一个字符串字面量，它可以被自动转换为 `std::string` 类型，而 `std::ostream` 已经定义了 `<<` 操作符的重载来输出 `std::string` 类型的数据。

//
```

> 因为对于基本数据类型（如 int，float，double，char 等）和一些标准库类型（如 `std::string`），`std::ostream `已经定义了 `<<` 操作符的重载，所以我们可以直接使用 `std::cout << `来输出这些类型的数据。

在C++中，操作符重载可以被视为一种特殊的函数。操作符的左侧和右侧的参数可以被视为这个函数的参数。

对于二元操作符（如 `+`，`-`，`*`，`/`，`<<` 等），它们有两个操作数：左操作数和右操作数。在操作符重载函数中，这两个操作数被视为函数的两个参数。

```cpp
std::cout << Vector2 << std::endl; // 将std::cout这个stream类型和Vector类型作为参数，将Vector中的数据输出到stream流中后，返回steram,以便后续继续使用<<.
```

如何知道操作符重载后的参数，对应的是右操作数还是左操作数？如果只有一个参数?

- 如果操作符重载是作为成员函数实现的，那么这个函数的隐含参数（也就是调用这个成员函数的对象）就是操作符的左操作数。这个函数的显式参数（也就是函数参数列表中的参数）就是操作符的右操作数。

- 如果操作符重载是作为非成员函数（通常是友元函数）实现的，那么这个函数的两个参数就分别是操作符的左操作数和右操作数。

## 42.this关键字

> C++中的this关键字，可以通过它来访问成员函数，成员函数就是某个类的方法。

- `this` 在一个 `const` 函数中，是一个 `const Entity* const` 或者 `const Entity*` 类型.
- 在函数内部可以使用 `this` 关键字，`this` 就是指向这个函数所属的当前对象实例的指针。

当我们想要在类中调用外部函数，且这个函数接受一个Entity类型作为参数时，就可以使用 `this` 。

```cpp
class Entity; // 前置声明
void PrintEntity(Entity* e);

class Entity
{
public:
	int x, y;
	Entity(int x, int y)
	{
		this->x = x;
		this->y = y;
		Entity& e = *this; // 非const函数下，就可以给e赋值。
		PrintEntity(this); //想要在类里调用这个函数
	}
};
void PrintEntity(Entity* e)
{

}
// 如果相传入一个常量引用，那么需要解引用this
void PrintEntity(const Entity& e);

class Entity
{
public:
	int x, y;
	Entity(int x, int y)
	{
		this->x = x;
		this->y = y;
		PrintEnity(*this);
	}
};

void PrintEntity(const Entity& e)
{

}
```

## 43. 对象生存周期

作用域可以是任何东西，比如说，函数作用域，还有if语句作用域，或者for和while循环作用域，或者空作用域、类作用域。

```cpp
// 类作用域上的变量在类对象结束时就被摧毁了：
class Entity
{
private:
	int x;
public:
	Entity()
	{
		std::cout << "Created Entity" << std::endl;
	}

	~Entity()
	{
		std::cout << "Destoryed Entity" << std::endl;
	}
};
// 当我们在一个作用域创建Entity对象时：
int main()
{
	{
		Entity e; // 函数在这里时，输出 "Created Entity"
		Entity* e = new Entity(); // 如果是在堆上建立，只会输出"Created Entity",而没有"Destroyed Entity"
	}
	std::cin.get(); // 在这里输出 "Destoeyed Entity"
}
```

每当我们在C++中进入一个作用域时，就是在push栈帧，不一定是一个栈帧。

> 当我在push数据时，就相当于把一本书放在书堆上，在这个作用域声明的变量的就是在这本书里面写的内容，一旦作用域结束，就把这本书从书堆上拿出来了，它也就结束了。
> _每个基于栈的变量，也就是你在这本书里创建的对象就结束了。_

创建错误用法：

```cpp
int* CreateArray()
{
	int array[50]; // 只是在栈上创建了，array是一个指向栈内存的指针。
	return array;// 这里是错误的，因为在函数结束时，array数组已经被释放了。
}
// 正确的创建方式有两种：
// 1. 直接上在函数内用 new 关键字创建一个在堆上的数组：
int* CreateArray()
{
	int* array = new int[50];
	return array;
}
// 2. 传入一个数组指针作为参数，函数体内部只是为了填充数组：
int array[50];
void CearteArray(int* array)
{
	// fill in the array;
}
```

作用域指针 - 本质上是一个类，一个指针的包装器。**在构造时在堆上分配指针，然后在析构时删除指针**

```cpp
// 作用域指针的实现：
class ScopedPtr
{
private:
	Entity* m_Ptr;
public:
	Entity(Entity* ptr) // 创建对象时，传入一个分配在堆上的Entity指针(new Entity())
		: m_Ptr(ptr)
		{
		}
	~Entity()
	{
		delete m_Ptr; // 对象结束时，释放在堆上的指针。
	}
};

int main()
{
	{
		Entity* e = new Entity();
		ScopedPtr e = new Entity();	 // ScopedPtr e(new Entity()); // 该变量在堆上创建，当ScopedPtr对象结束时，自动释放内存。
	}

std::cin.get();
}
```

> 这种可以自动构造，自动析构，离开作用域之后就自动销毁的栈变量是非常有用的：
> 比如，写一个计时器，计算你想要的在基准测试范围内的时间：
> _写一个time类，在对象创建构造时开始计时，在对象销毁时停止计时，并打印出计时；_

## 44. 智能指针

    智能指针就是用来自动实现，当new在堆上分配内存时，可以自动调用delete来释放内存。

> 智能指针就是对原始指针的包装，当创建一个智能指针时，会调用new为你分配内存，然后基于你使用的智能指针，会在设定好的时刻自动释放内存。

_要访问智能指针, 首先要做的就是包含memory头文件_ : `#include <memory>`

### unique_ptr 指针

最简单的智能指针：`unique_ptr` , 这是一个作用域指针，也就是说当作用域结束时，会自动调用 `delete` 来释放内存。
_unique_ptr_ 是不能复制的，因为如果复制了就相当于有两个ptr指向同一块内存，而当其中一个ptr释放内存后，另一个ptr就指向了被释放后的内存。

所以说：_unique_ptr 是当你想要一个作用域指针时，对你真正想要的指针的**唯一**引用_

- 优先使用 `unique_ptr`, 因为它有一个较低的开销，但如果需要在对象之间共享，不能使用 unique_ptr的时候，就使用`shared_ptr

- `unique_ptr` 构造函数实际上是 explicit的， **没有构造函数的隐式转换，需要显式调用构造函数**。

在使用`unique_ptr`的时候,有两种选择:
第一种选择:

```cpp
std::unique_prt<Entity> entity(new Entity()); // 这样不太安全,在构造函数发生异常时,会得到一个没有引用的空指针,容易造成内存泄漏.
```

第二种选择:

```cpp
std::unique_ptr<Entity> entity = std::make_unique<Entity>(); // 这样更加安全,不会造成内存泄漏.
```

- `unique_ptr` 优先使用，因为它开销很低，只是一个栈分配对象，当栈分配对象死亡时，就会调用deletes 释放内存。

### 共享指针 shared_ptr

当想要共享指针时，才使用 `shared_ptr` 。

> shared_ptr 的实现方式取决于使用的编译器和编译器中的标准库。

`shared_ptr` 的工作方式是通过引用计数的，引用计数就是一种跟踪统计你的指针有多少引用的方法, 一旦引用计数为0，这个指针就会被删除。

```cpp
std::shared_ptr<Entity> sharedEntity = std::make_shared<Entity>();

std::shared_ptr<Entity> sharedEntity(new Entity()); // 也不建议, 因为shared_ptr会分配一块叫控制块的内存,用来存储计数,// new Entity() 也会分配一次内存, 造成效率降低.

std::shared_ptr<Entity> e0 = sharedEntity; // 共享指针可以复制;
```

### weak_ptr

> weak_ptr 可以和 shared_ptr一起使用，但是不会增加引用计数。

```cpp
std::weak_ptr<Entity> weakEntity = sharedEntity; // 可以复制，但是不会增加引用次数。
```

```cpp
{
	std::weak_ptr<Entity> e1;
	{
		std::shared_ptr<Entity> sharedEntity = std::make_shared<Entity>();
		e1 = sharedEntity;
	} // 这时析构函数被调用，内存释放.
}
```

## 45. C++的拷贝和拷贝构造函数

> 当使用 = 操作符时，赋值操作符，将一个变量设置为另一个变量时，总是在复制值（引用除外）

浅拷贝，当我们用 = 赋值一个类时，它会将类的所有成员变量和方法复制一份，如果类中有指针，那么就只是将指针所存储的内存地址复制了一份，也就是说指向的还是同一个地址。

```cpp
#include <iostream>
#include <cstring>

class String
{
private:
    char* m_Buffer;
    unsigned int m_Size;
public:
    String(const char* string)
    {
        m_Size = strlen(string);
        m_Buffer = new char[m_Size + 1];
        memcpy(m_Buffer, string, m_Size);
        m_Buffer[m_Size] = 0;
    }
    friend std::ostream& operator<<(std::ostream& stream, const String& string);

    ~String()
    {
        delete[] m_Buffer;
    }
    char& operator[](unsigned int index)
    {
        return m_Buffer[index];
    }

};


std::ostream& operator<<(std::ostream& stream, const String& string)
{
    stream << string.m_Buffer;
    return stream;
}

int main()
{
    String string = "Cherno"; // 隐式转换 String string("Cherno")
    String second = string; // 当我们使用 = 赋值时，将string中的所有成员变量和函数都复制到了一个新的内存地址。
    // 所以，里面的指针，m_Buffer也会被复制，也就是这两个m_Buffer是同一个内存地址，当作用域结束时，析构函数就会释放两次这个内存，导致程序崩溃。
    string[2] = 'a';
    std::cout << string << std::endl;
    std::cout << second << std::endl;

    std::cin.get();
}
```

输出结果：

```sh
Charno
Charno
```

**拷贝构造函数** ，如果我们想要深拷贝

> 拷贝构造函数也是一个构造函数， 比如当你复制一个字符串的时候，它就被调用。
> 当你把一个字符串赋值给一个对象时（也是一个字符串）,当你试图创建一个新的变量，并给它分配另一个变量（和你正在创建的变量有相同的类型）时，就复制这个变量。

```cpp
String(const String& other)
	: m_Buffer(other.m_Buffer), m_Size(other.m_Size)
	{} // 这样不可以，因为我们不仅仅想复制指针，还想复制指针所指向的内存。
String(const String& other)
{
	memcpy(this, &other, sizeof(String));
}
```

**深度拷贝**

```cpp
String(const String& other)
	: m_Size(other.m_Size)
{

	m_Buffer = new char[m_Size + 1]; // 先划分一块新的地址
	memcpy(m_Buffer, other.m_Buffer, m_Size + 1; // 把要赋值的对象中m_Buffer成员指向内存中的内容复制过来。
}
```

如果使用这样的 `PrintString` 函数，会造成资源浪费：

```cpp
void PrintString(String string)
{
	std::cout << string << std::endl;
}
```

函数传参也是复制传递，导致每次都要再调用一次构造拷贝函数。
更好地选择是：

```cpp
void PrintString(const String& string)
{
	std::cout << string << std:endl;
}
```

     记得总是通过 `const` 来应用对象 。

拷贝构造函数的格式：

```cpp
// 声明
T(const T& var)；
// 定义
T(const T& var)
{
	// 函数体进行深拷贝
	// 1. 分配空间
	// 2. 放副本
}
// 不使用拷贝函数，禁止赋值：
T(const T& var) = delete;
```

- 每当你编写一个变量被赋值另一个变量的代码时，你**总是**在复制。在指针的情况下，你在复制指针，也就是内存地址，内存地址的数字，就是数字而已，而不是指针指向的实际内存。
- "成员树" 不包含指针和引用时，浅拷贝和深拷贝没有区别。
  _浅拷贝应该指用来拷贝基础数据类型(非指针变量)_

## 46. C++箭头操作符

1. **特点**

- 箭头运算符必须是类的成员。
- 一般将箭头运算符定义成了const成员，这是因为与递增和递减运算符不同，获取一个元素并不会改变类对象的状态。

2. **对箭头运算返回值的限定**
   > 箭头运算符的重载永远不能丢掉成员访问这个最基本的含义，当我们重载箭头时，可以改变的是箭头从哪个对象当中获取成员，而箭头获取成员这一事实不变。

重载的箭头运算符**必须返回类的指针或者自定义了箭头运算符的某个类的对象。**

**3.三种应用场景**

1)可用于指针调用成员：p->x 等价于 (\*p).x

2)重载箭头操作符

```cpp
#include <iostream>
class Entity
{
private:
    int x;
public:
    void Print()
    {
        std::cout << "Hello!" << std::endl;
    }
};
class ScopedPtr
{
private:
    Entity* m_Ptr;
public:
    ScopedPtr(Entity* ptr)
        : m_Ptr(ptr)
    {
    }
    ~ScopedPtr()
    {
        delete m_Ptr;
    }
    Entity* operator->()  //重载操作符
    {
        return m_Ptr;
    }
};

int main()
{
    {
        ScopedPtr entity = new Entity();
        entity->Print();
    }
    std::cin.get();
}
```

进一步,可以写为const版本的：

```cpp
#include <iostream>
class Entity
{
private:
    int x;
public:
    void Print() const   //添加const
    {
        std::cout << "hello!" << std::endl;
    }
};

class ScopedPtr
{
private:
    Entity* m_Ptr;
public:
    ScopedPtr(Entity* ptr)
        : m_Ptr(ptr)
    {
    }
    ~ScopedPtr()
    {
        delete m_Ptr;
    }
    Entity* operator->()
    {
        return m_Ptr;
    }
    const Entity* operator->() const //添加const
    {
        return m_Ptr;
    }
};

int main()
{
    {
        const ScopedPtr entity = new Entity(); //如果是const，则上面代码要改为const版本的。
        entity->Print();
    }
    std::cin.get();
}
```

3. 用于计算成员变量的 `offset`
   > 因为“指针->属性” 的方法实际上就是通过把指针的指和属性的偏移量相加，得到属性的内存地址进而实现访问。而把指针设为 `nullptr` (0), 然后 `->` 属性就等于 0 + 属性偏移量。而类的结构已经写了出来 (float x, y, z) ,float 4字节，所以，它在编译时就知道编译量(0, 4, 8), 所以无关对象是否创建。

_`->` 的含义就是 把指向类的指针所存的地址 + 成员变量的偏移量，得到成员变量地址再进行访问。_

如何使用箭头操作符，来获取内存中某个成员变量的偏移量？

```cpp
size_t offset = (size_t)&((Vector3*)0)->z;
```

## 47. C++的动态数组（std::vector）

> 容器包含的数据类型，实际上由你决定，所有东西由模板组成。

1. `vector 本质上是一个动态数组，是内存连续的数组`
2. 它的使用需要包含头文件： `#include <vector>`
3. 使用格式: **类型；尽量使用对象而非指针**

```cpp
std::vector<T> a; // T是一种模板类型，尽量使用对象而非指针；
```

4. 添加元素

```cpp
a.push_back(element);

// 定义一个类
struct Vertex
{
	float x, y, z;
}
std::verctor<Vertex> vertices; // 定义一个Vertex类型的动态数组
vertices.push_back({1, 2, 3});
vertices.push_back({4, 5, 6});
```

5. for 循环遍历

```cpp
// 第一种方式
for (int i = 0; i < vertices.size, i++)
{
	std::cout << vertices[i] << std::endl;
}
//第二种方式
for (Vertex& v : vertices) // vertices中的每一个元素，被Vertex类型的变量 v 引用。
{
	std::cout << v << std::endl;
}
```

6. 清除数组列表

```cpp
vertices.clear();
```

7. 擦出指定元素

```cpp
vertices.erase(vertices.begin() + 1); // 参数是迭代器类型
```

8.  当将这些vector传递给函数、类或者其他东西时，记得使用引用。以确保没有把整个数组复制到函数中

```cpp
void Function(const std::vector<Vector>& vertices) // 如果不需要修改数组，请加上const引用
{

}
```

## 48. C++的std::vector使用优化

> vector的工作方式：
>
> 1.  创建一个vector, 然后push_back元素，也就是向数组中添加元素;
> 2.  如果vector的空间不够，不能容纳新的想要的元素时, 就需要重新分配新的内存，至少足够容纳这些想要加入的新元素。
> 3.  而**当前vector的内容，从内存中的旧位置复制到内存中的新位置，然后删除旧位置的内存。**
>     _这将是一个缓慢的操作，我们需要复制所有的元素，再重新分配。_

    这就是今天的优化策略，如何避免复制对象，特别是vector对象？

如果不优化：

```cpp
#include <iostream>
#include <string>
#include <vector>


struct Vertex
{
    float x, y, z;
    Vertex(float x, float y, float z)
        : x(x), y(y), z(z)
        {

        }
    Vertex(const Vertex& other)
        : x(other.x), y(other.y), z(other.z)
    {
        std::cout << "Copied!" << std::endl;
    }
};

int main()
{
    std::vector<Vertex> vectices;
    vectices.push_back(Vertex{1, 2, 3}); // 当创建了一个Vertex类对象，然后将其放入vector时，产生了一个复制
    vectices.push_back(Vertex{4, 5, 6}); // 这时直接产生了两个复制，因为会把之前的第一个Vertex对象也复制一次. 再加上自己传入一次
    vectices.push_back(Vertex(7, 8, 9)); // 到这里时，会复制前面的两个，然后自己传入一次。

    std::cin.get();
}
```

结果：

```sh
Copied!
Copied!
Copied!
Copied!
Copied!
Copied!
```

第一次 `push_back` 产生一个复制，第二次 `push_back` 产生两个复制，第三次 `push_back` 产生三次复制。

**第一步优化策略**: 直接告诉 `vector` 我们需要三个对象的空间大小, 然后再 `push_back`：

```cpp
vertices.reserve(3); // 解决了每次内存不够，分配新内存，而把前面的都复制一遍再重新放入新内存的问题
vertices.push_back(Vertex{1, 2, 3});
vertices.push_back(Vertex{4, 5, 6});
vertices.push_back(Vertex{7, 8, 9});
```

结果：

```sh
Copied!
Copied!
Copied!
```

只有三次复制，每传入一个，就复制一次。

我们在使用了 `vertices.reserve(3)` 后仍然有三个复制，那是因为这个 vertex其实是在 main函数的栈上构造的，然后复制到实际的vector中。

**第二步策略**：避免上述情况，直接在实际的vector中构造, 可以使用 `emplace_back`, 而不是 `push_back`.

```cpp
vectices.emplace_back(1, 2, 3); // 将不是传递已构建的vertex对象，而是传递构造函数的参数列表；也就是传递给构造函数的参数。
```

结果显示，没有一个复制了。

_这次优化甚至不需要很长时间，只要**知道实际发生了什么**，并**了解我们的环境**，知道如何**使用可用的工具**来优化它_

## 49. C++中使用库（静态链接）

> 只是想克隆存储库，然后编译和运行就可以！
> 因此，倾向于，在实际解决方案中的实际项目文件夹中，保留使用的库的版本。

**以上主要是以二进制文件的形式进行链接，而不是获取实际依赖库的源代码并自己进行编译。**

    静态链接意味着这个库会被放在你的可执行文件中，它在你的exe文件中, 或其他系统下的可执行文件中。

    动态链接库是在运行时被链接的。比如 dll文件，就是动态链接库。

- 头文件通过提供声明，告诉我们哪些函数是可用的
- 然后库文件为我们提供了定义，这样就可以连接到那些函数，并在C++中调用函数。

## 52. C++中如何处理多返回值

**方法一：通过函数参数引用或指针的方式**

> 把函数定义成 void,然后通过**参数引用传递**的形式“返回”两个字符串，这个实际上是修改了目标值，而不是返回值，但某种意义上它确实是返回了两个字符串，而且没有复制操作，技术上可以说是很好的。但这样做会使得函数的形参太多了，可读性降低，有利有弊

````cpp
```cpp
include <iostream>
void GetUserAge(const std::string& user_name,bool& work_status,int& age)
{
    if (user_name.compare("xiaoli") == 0)
    {
        work_status = true;
        age = 18;
    }
    else
    {
        work_status = false;
        age = -1;
    }
}

int main()
{
    bool work_status = false;
    int age = -1;
    GetUserAge("xiaoli", work_status, age);
    std::cout << "查询结果：" << work_status << "    " << "年龄：" << age << std::endl;
    getchar();
    return 0;
}
````

``

**方法二：通过函数的返回值是一个array或者vector**

> 也可以返回一个vector，同样可以达成返回多个数据的目的。 不同点**是Array是在栈上创建，而vector会把它的底层储存在堆上**，所以从技术上说，返回Array会更快 但以上方法都**只适用于相同类型**的多种数据的返回

```cpp
std::array<std::string, 2> ChangeString()
{
    std::string a = "1";
    std::string b = "2";

    std::array<std::string, 2> result;
    result[0] = a;
    result[1] = b;
    return result;
}
```

**方法三：使用`std::pair` 返回两个返回值**

> 可以**返回两个不同类型**的数据返。  
> 使用std::pair这种抽象数据结构，该数据结构可以绑定两个异构成员。这种方式的弊端是**只能返回两个值。**

```cpp
#include <iostream>

std::pair<bool, int> GetUserAge(const std::string& user_name)
{
    std::pair<bool, int> result;
    if(!user_name.compare("xiaoli"))
    {
        result = std::make_pair(true, 18);
    }
    else
    {
        result = std::make_pair(false, -1);
    }
    return result;
}

int main()
{
    std::pair<bool, int> result = GetUserAge("xiaoli");
    std::cout << "查询结果：" << result.first << "  "
              << "年龄： " << result.second << std::endl;
    std::cin.get();
}
```

**方法四：使用 `std::tuple` 返回三个或者三个以上返回值**

> std::tuple这种抽象数据结构可以将三个或者三个以上的异构成员绑定在一起，返回std::tuple作为函数返回值理论上可以返回三个或者三个以上的返回值。  
> `tuple`相当于一个类，它可以包含x个变量，但他不关心类型，用`tuple`需要包含头文件`#include`

```cpp
#include <iostream>
#include <tuple>

std::tuple<bool, int,int> GetUserAge(const std::string& user_name)
{
    std::tuple<bool, int,int> result;

    if (user_name.compare("xiaoli") == 0)
    {
        result = std::make_tuple(true, 18,0);
    }
    else
    {
        result = std::make_tuple(false, -1,-1);
    }

    return result;
}

int main()
{
    std::tuple<bool, int,int> result = GetUserAge("xiaolili");

    bool work_status;
    int age;
    int user_id;

    std::tie(work_status, age, user_id) = result;
    std::cout << "查询结果：" << work_status << "    " << "年龄：" << age <<"   "<<"用户id:"<<user_id <<std::endl;
    getchar();
    return 0;
}
```

**方法五： 返回一个结构体（推荐）**

> 结构体是在栈上建立的，所以在技术上速度也是可以接受的  
> 而且不像用pair的时候使用只能`temp.first, temp.second`，这样不清楚前后值是什么，可读性不佳。而如果换成`temp.str, temp.val`后可读性极佳，永远不会弄混！

```cpp
#include <iostream>
struct result {
    std::string str;
    int val;
};
result Function () {
    return {"1", 1};//C++新特性，可以直接这样子让函数自动补全成结构体
}
int main() {
    auto temp = Function();
    std::cout << temp.str << ' ' << temp.val << std::endl;
}
--------------------------------------------
#include <iostream>
using namespace std;

struct Result
{
    int add;
    int sub;
};

Result operation(int a, int b)
{
    Result ret;
    ret.add = a + b;
    ret.sub = a - b;
    return ret;
}

int main()
{
    Result res;
    res = operation(5, 3);
    cout << "5+3=" << res.add << endl;
    cout << "5-3=" << res.sub << endl;
}
```

## 53. C++的模板

**模板**： 模板允许你定义一个可以根据你的用途进行编译的模板（有意义下）,故所谓模板，就是让编译器基于DIY的规则为你写代码。
**函数的模板（形参）**

- 不使用模板

```cpp
  void Print(int temp) {
      cout << temp;
  }
  void Print(string temp) {
      cout << temp;
  }
  void Print(double temp) {
      cout << temp;
  }
  int main() {
      Print(1);
      Print("hello");
      Print(5.5);
      //如果要用一个函数输出三个类型不同的东西，则要手动定义三个不同重载函数
      //这其实就是一种复制粘贴就可以完成的操作
  }
```

> 使用模板
> 格式： `template<typename T>`

```cpp
template<typename T>
void Print(T temp)
{
	cout << temp;
}
int main()
{
	Print(5); // 这时其实也隐式地得到了输入参数的类型是int,相当于下面这种写法：
    Print<int>(5); // 在尖括号中输入int,也就是说T = int, T被替换成了int.
    Print("Hello");
}
```

**值得注意的是** ： 通过 `template` 定义，说明定义的函数只是一个模板，在编译时会被评估，是否被调用了。只有当调用函数时，模板函数才会基于传递的模板参数被真正的传建。

**类的模板**

> 传递数字给模板，来指定要生成的类

```cpp
template<int N> class Array {
private:
    //在栈上分配一个数组，而为了知道它的大小，要用模板传一个数字N过来
    int m_Array[N];
};
int main() {
    Array<5> array;//用尖括号给模板传递构造的规则。
}
```

> 传多个规则给模板，**用逗号隔开就行**

```cpp
//两个模板参数：类型和大小
template<typename T, int size> class Array {
private:
    T m_Array[size];
};
int main() {
    Array<int, 5> array;
}
```

哪些地方可以用模板，哪些地方不可以用模板？

> 模板在日志系统或者其他的使用场景下, 你想记录每一种可能的类型，将每个函数都重载太麻烦了, 这些可以让编译器基于你设置的规则为你编写代码.
> 但是，当模板非常复杂的时候，很难去阅读和理解那些代码被编译了那些没有。

## 54. 堆和栈内存的比较

> 栈通常是一个预定义大小的内存区域，通常为2兆字节左右。

> 堆也是一个预定了默认值的区域，但是它可以生长。

当我们在栈中分配变量时，发生的是，栈指针在移动，也就是栈顶部的指针，然后返回栈指针的地址。
当超出作用域时，栈内存就会被释放，也就是说栈中的元素会被全部弹出，栈指针回到作用域开始的地方。

大部分栈实现中，栈是倒着来的，高地址存储的是栈底的数据。

**堆和栈的内存分配方式不同：**
对栈来说：

> **在栈上**，分配的内存都是**连续**的。添加一个int，则**栈指针（栈顶部的指针）** 就移动4个字节，所以连续分配的数据在内存上都是**连续**的。栈分配数据是直接把数据堆在一起（所做的就是移动栈指针），所以栈分配数据会很快 。  
> 如果离开作用域，在栈中分配的所有内存都会弹出，内存被释放。

对堆来说：

> **在堆上**，分配的内存都是**不连续**的，`new`实际上做的是在内存块的**空闲列表**中找到空闲的内存块，然后把它用一个指针圈起来，然后返回这个指针。（但如果**空闲列表**找不到空闲的内存块，则会询问**操作系统**索要更多内存，而这种操作是很麻烦的，潜在成本是巨大的）  
> 离开作用域后，堆中的内存仍然存在.

建议： **能在栈上分配就在栈上分配**，**不能够在栈上分配时或者有特殊需求时（比如需要生存周期比函数作用域更长，或者需要分配一些大的数据），才在堆上分配**
栈上分配的东西都是连续的，因为可以放到CPU缓存线（Cache Line可以理解为CPU Cache的最小缓存单位）上。
CPU要访问的数据如果在Cache中，称为 “hin”, 反之则称为 “Miss”.

_在栈上分配就像一条CPU指令，非常快。而在堆上分配，底层 `malloc` 函数要做一堆事儿_

## 55.C++中的静态数组

> 本质上`std::array` 和 C中的普通数组没有区别，都是存储在栈中，都是限定长度的。

**使用 `std:array` 数组的优势是：**
作为STL标准库的一部分，`std::array` 数组可以作为一个迭代器使用 `array.begin(), array.end()` 等等，也可以使用像 `array.size()` 这样的方法。
因为 std::array 是一个标准库中的类，可以使用大量的STL算法。

还有一个优势就是 `std::array` 在调试的时候可以设置边界检查，比如 `std::array<int, 5> data` , 当写 `data[5] = 0;` 是不行的。而普通的C数组没有，它会覆盖还没有分配的数据。

实际上 `std::array<int, 5> data; data.size()` 中的 `size()` 方法返回的不是一个 `size` 变量或者什么东西，只是返回5，因为这个size就是输入的模板参数 `<int ,5>`。

## 56. C++函数指针

> 函数指针，就是将一个函数（函数名）赋值给一个变量（指针变量）的方法。

`auto` 关键字在函数指针中是非常有用的：

```cpp
void Hellword() {
	std::cout << "Hello World" << std::endl;
}
auto function = HellWorld(); // error 因为这是函数的返回值void类型
auto function = HellWorld; // OK 函数名其实就是指针类型，而不是调用了函数。
auto function // 等价于 void(*function)()
```

这里的 `function` 的类型是 `void(*function)();` 空函数指针类型。
`function` 只是 `void(*)()`类型的变量名。
``
函数指针还可以用来批量运行其他函数的函数：

```cpp
void PrintValue(int value) {
	std::cout << "Value: " << value << std::endl;
}
void ForEach(const std::vector<int> &values, void(*func)(int)) {
	for (int value : values) {
		func();
	}
}

std::vector<int> data = {1, 2, 3, 4, 5};
ForEach(data, PrintValue);
```

这里的 `PrintValue` 函数比较简单，也可以用 `lambda` 表达式来实现：
`[](int value) { std::cout << "Value: " << value << st::endl; }`
`[]` 的意思就是捕获方式，即如何传入传出参数。

## 59. C++中的Lambda

1. Lambda本质上是一个匿名函数，用这种方式创建函数不需要实际创建一个函数，它就像一个快速的一次性函数。lamda更像是一种变量，在实际编译的代码中作为一个符号存在，而不是正式的函数那样。
2. 使用场景:
   > 在我们会设置函数指针指向函数的任何地方，我们都可以将他设置为lambda。

3.Lambda表达式的写法（使用格式）：`[ ] (参数表) {函数体}`

> 中括号表示的是捕获，作用是如何传递变量lambda使用外部（相对）的变量，就要使用 **捕获**。

如果使用捕获，则：

- 添加头文件 `#include <functional>`;
- 修改相应的函数签名：`std::function <void(int)> func` 替代 `void(*func)(int)`
- 捕获的几种使用方式：
  > [=] , 则复制传递所有在lambda表达式中使用的变量，比如 `x, y, z` 三个值，只有x,y在lambda表达式中被使用了，则z虽然也在lambda的作用域中，但是不会被复制传递。
  > [&] , 则引用传递所有在lambda表达式中使用的变量。
  > [a] 或者 [&a] , 则是将单个a复制或引用传递。

lamda表达式在引用传递到 `ForEach(values, lambda)` 时，函数的参数lambda是一个临时对象，因为它是引用传递的，相当于 `ForEach(values, [=](int value) {std::cout << "Value: " << a << std::endl;})` ,所以其实传入的是一个临时变量。

**对于临时对象，不可以绑定到非常量引用上，因为非常量引用可以修改它所引用的对象，但是临时对象在表达式结束后就被销毁了，所以不能修改它，因为值可以绑定到常量引用上**

`mutable` 在lambda的作用是**允许lambda函数体**修改通过拷贝传递捕获的参数。若我们在lambda中给a赋值会报错，需要写上mutable 。

```cpp
nt a = 5;
auto lambda = [=](int value) mutable { a = 5; std::cout << "Value: " << value << a << std::endl; };
```

另一个使用lambda的场景`find_if`

> 我们还可以写一个lambda接受vector的整数元素，遍历这个vector找到比3大的整数，然后返回它的迭代器，也就是满足条件的第一个元素。  
> **`find_if`**是一个搜索类的函数，区别于`find`的是：**它可以接受一个函数指针来定义搜索的规则，返回满足这个规则的第一个元素的迭代器**。这个情况就很适合lambda表达式的出场了

```cpp
#include <algorithm>
#include <vector>
#include <iostream>

int main() {
    std::vector<int> values = { 1, 5, 3, 4, 2 };
    //下面就用到了lambda作为函数指针构成了find_it的规则
    auto it = std::find_if(values.begin(), values.end(), [](int value) { return value > 3; });  //返回第一个大于3的元素的迭代器
    std::cout << *it << std::endl;  //将其输出
}
```

[[Lambda深入理解]]

## 60. 为什么不使用 `using namespace std`?

1. 不容易区分各个函数的来源
   > 比如位我定义了一个自己的 `vector`, 而标准库里也有一个 `vector` , 那么使用了 `using namespace std` 之后怎么知道这个 `vector` 来自那里。

```cpp
std::vector<int> vec1;
DiyClass::vertor<int> vec2;
using namespace std;
using namespace DiyClass; // 如果别人突然用了这个命名空间
vector<int>vec3 // 就会有歧义。
```

2. 可能会产生运行的错误，编译器不会报错，即 `0 error, 0 warning`

```cpp
using namespace apple {
	void print(const char* s) {
		std::string temp = text;
		std::reverse(temp.begin(), temp.end());
		std::cout << temp << std::endl;
	}
}

using namespace orange {
	void print (std::string& s) {
		std::cout << s << std::endl;
	}
}

int main() {
	using namespace apple;
	using namespace orange;
	// 会偷偷地调用来自 orange 的函数，而且不会报错，因为传入的是 “Hello” 是const char*类型
	print("Hello"); // 0 error, 0 warning 的错误
}
```

3. 一定不要在头文件内使用 `using namespace std`
   > 如果别人用了你的头文件，就会把这些命名空间用在了你原本没有打算用的地方，会导致莫名其妙的产生bug。
4. 可以在一个小作用域里用，_但能不用就不用_；

### C++的命名空间

1. 命名空间是C++独有的，C是没有的，命名空间的目的就是为了防止命名冲突，因为C有命名冲突的风险。
2. **类本身就是命名空间**
   > 类外时用一个类里面的成员时需要加是时小虚需要徐亚需要较即加 ::是时小虚需要徐亚需要较即加 ::是时小虚需要徐亚需要较即加 ::
3. 命名空间的用法

```cpp
using namespace apple; // 在其所在作用域中，空间内的函数不需要加 ::
using apple::print; // 单独声明某个函数

namespace a = apple; // 给一个命名空间起个别名
a::print("Hello"); // 就可以这样了
```

> 命名空间结束后无需分号;

```cpp
namespace foo {
	class Bar {};
}
```

_在使用像namespace a = ... 或者 using namespace apple 时，请在一个尽量小的作用域中使用_ 4. **命名空间可以不连续**

> “命名空间可以不连续”意味着你可以**在不同的文件中定义同一个命名空间的内容**，而不需要这些定义在物理位置上是连续的。

5. 模板特例化
   > 在命名空间中，模板特例化必须在原始模板所属的命名空间中声明，只要在命名空间中做了声明，就能在命名空间外部取定义它

```cpp
// 必须将模板特例化声明成std的成员
namespace std {
	template <> struct hash<Foo>;
}

// 在std中添加了模板特例化的声明后，我们就可以在命名空间std的外部进行定义了
template<> struct std::hash<Foo> {
	size_t operator()(const Foo& f) const {
		return hash<string>()(f.str) ^
		hash<doubble>()(f.d);
	}
}
```

6. 全局命名空间
   全局作用域中定义的名字（即在所有类、函数以及命名空间之外定义的名字）也就是定义在全局命名空间 `global namespace` 中。**全局作用域是隐式的** ，所以全局命名空间没有名字。可以用下列形式表示全局命名空间中的一个成员：

```cpp
::member_name;
```

7. 内联命名空间
   > `inline` 关键字的用法：`inline` 用于提示编译器将函数的代码直接插入到调用点，而不是常规的函数调用，从而减少了调用开销；
   > `inline` 常用于 函数定义前、类的成员函数定义前；
   > \*\*注意，inline函数的定义必须在每个使用它的翻译单元可见，因此常定义在头文件中“

C++11新标准引入了一种新的嵌套命名空间，称为内联命名空间`inline namespace`。**内联命名空间可以被外层命名空间直接使用。**

> 定义内链命名空间的方式是在关键字 `namespace` 前添加关键字 `inline`:
> **`inline` 必须在命名空间第一次出现的地方（同一个翻译单元内）** ，而因为命名空间可以不连续，所以尽量把内链函数或**内链命名空间**放在头文件中，以确定每个使用这些 `inline` 函数的翻译单元中都包含它们的定义且都是第一次出现的地方。

```cpp
inline namespace FifthEd {

}

// 后续再打开命名空间的时候inline就可写可不写了
namespace FifthEd {

}
```

内联命名空间的用法：

> 当应用程序的代码在一次发布和另一次发布之间发生改变时，常使用内联命名空间。例如我们把第五版`FifthEd`的所有代码放在一个内联命名空间中，而之前版本的代码都放在一个非内联命名空间中：

```cpp
namespace FourthEd {
	class Cat {  };
}

namespace foo {
#include "FifthEd.h"
#include "FourthEd.h"
}
```

因为`FifthEd` 是内联的, 所以 `foo::` 就可以直接调用 `FifthEd` 的成员，而如果想用到早期版本的代码，则必须像其他嵌套命名空间一样加上完整的外命名空间名字：

```cpp
foo::FourthEd::Cat;
```

7. 未命名空间
   > 关键字`namespace`后紧跟花括号括起来的一系列声明语句就是未命名空间。和其他命名空间不同，未命名的命名空间仅在特定的文件内部有效，其作用范围不会横跨多个不同的文件。

_可以理解为，未命名空间就相当于C中的全局静态变量_

未命名的命名空间中定义的变量具有静态生命周期：它们在第一次使用前被创建，直到程序结束时才销毁。**相当于C中的静态声明。**

要注意的是：如果未命名的命名空间定义在文件的最外层作用域中，则该命名空间一定要与全局作用域中的名字有所区别：

```cpp
// i的全局声明
int i;
// i在未命名的命名空间中的声明
namespace {
    int i;
}
// 二义性错误: i的定义既出现在全局作用域中, 又出现在未嵌套的未命名的命名空间中
i = 10;
```

## 62. C++的线程

1. 使用多线程，首先要添加头文件 `#include <thread>`。
2. 在 **Linux平台下**编译时需要加上 “-lpthread” 链接库
   cmake编译的情况下：

```txt
find_package(Threads REQUIRED)
target_link_libraries(LearnCpp PRIVATE Threads::Threads)
```

3. 创建一个线程 `std::thread objName(一个函数指针以及其他可选的任何参数)`
4. 等待一个线程完成它工作的方法（确保子线程的工作已完成）`objName.join()`
   > 对于子线程，C++要求其就是`std::thread` **对象析构时必须确保线程被处理**（要么已结束`objName.join()`, 要么被分离 `objName.detach()`）

- 调用`.join()`会阻塞主线程，直到子线程完成执行。这种方式确保子线程在主线程继续执行之前已经结束
- 调用`.detach()`会将子线程与主线程分离，使子线程在后台独立运行。分离后的子线程不再与`std::thread`对象关联，主线程不会等待子线程结束。分离后的子线程会在完成后自动清理资源。
- 可以使用 `std::this_thread::sleep_for()` 来减缓cpu运行速度，不然cpu以最快速度运行，很快CPU占用率会达到100%。
  _多线程对于加速程序是十分游泳的，线程的主要目的就是优化_

## 63. C++的计时

> 在C++中，[`auto`](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)关键字用于自动类型推断, 只能用在在函数体内、初始化变量时或作为返回类型。
> 结构体或类（只是声明）的成员变量不能使用 `auto` 关键字进行初始化, 这是因为`auto`需要在编译时推导出类型，而成员变量的初始化是在构造函数执行时（调用时）进行的，这时候编译器已经完成了类型的推导。

缓冲区（Buffer）是计算机系统中用于临时存储数据的一块内存区域。它的主要目的是在数据传输过程中**平衡速度差异**，提高效率和性能。

在编程中，缓冲区广泛应用于各种场景，例如文件I/O、网络通信和标准输入输出。

如果只执行 `std::cout << "Hello, World!";` 而不写 `std::endl` 或 `\n`，输出到缓冲区的内容可能不会立即显示在控制台上。这是因为标准输出流（`std::cout`）通常是带缓冲的，**只有在缓冲区满了、遇到换行符、显式刷新缓冲区**（如调用 `std::flush`）或程序正常结束时，缓冲区的内容才会被输出到控制台。

**利用chrono类计时**

> 在`std::chrono`中，有三种主要的时钟类型：`system_clock`、`steady_clock`和`high_resolution_clock`。
> `system_clock`受系统时间的影响，可能因为夏令时或用户更改系统时间而变化；
> `steady_clock`提供一个稳定的时钟，其时间点之间的差异是持续且一致的，不会跳跃；
> `high_resolution_clock`是一个快的时钟，通常是`system_clock`或`steady_clock`中较优的一个(是一个`typedef`)。根据你的需求明确选择时钟类型

1. 包含头文件 `#include <chrono>`
2. 获取当地时间：

```cpp
auto start = std::chrono::high_resolution_clock::now();
auto end = std::chrono::high_resolution_clock::now();
```

3. 获取时间差：

```cpp
std::chrono::duration<float> duration = end - start;
```

写一个计时类，利用类的生存周期来计时：

```cpp
struct Timer{
	std::chrono::_V2::system_clock::time_point start, end;
	std::chrono::duration<float> duration;
	Timer() {
		start = std::chrono::high_resolution_clock::now();
	}
	~Timer() {
		end = std::chrono::high_resolution_clock::now();
		duration = end - start;
		float ms = duration.count() * 1000.0f;
		std::cout << "Timer took " << ms << "ms" << std::endl;
	}
};
void Functiuon() {
	Timer timer;
	for (int i = 0; i < 100; i++) {
		std::cout << "Hello\n" // \n 比std::endl快
	}
}
int main() {
	Function();
}
```

## 64. C++多维数组

> 二维数组可以理解为：有一个内存块，里面存放的是连续的指针，，每个指针都指向我内存中的某一个数组。
> 所以我么得到的就是指向数组的指针的集合，就是数组的数组。

_类型只是一种语法，设置类型是用来处理数据的_

```cpp
int** a2d = new int*[5];
for (int i = 0; i < 5; i++) {
	a2d[i] = new int[5]; // 这会分配5个单独的缓冲区，每个缓冲区有五个整数，被随机分配内存的空闲位置。
}
```

被分配的内存很分散，会造成内存碎片的问题，大量调用时，很可能造成 cache miss, 损失性能。

数组优化的一个方法:**把二维数组转化成一维数组来存储**。

```cpp
int *array = new int[5 * 5];
for (int x = 0; x < 5; x++) {
	for (int y = 0; y < 5; y++) {
		array[i * 5 + j] = 2;
	}
}
```

## 65. C++内置的排序函数

1. 添加头文件 `#include <algorithm>`
2. 使用规则

```cpp
sort(vec.begin(), vec.end(), 谓语)
```

> **谓语可以设置排序的规则**， 谓语可以是内值函数，也可以是lambda表达式

    如果不加第三个参数，默认是从小到大排序

```cpp
#include<iostream>
#include<vector>
#include<algorithm>

int main()
{
    std::vector<int>  values = {3, 5, 1, 4, 2};
    std::sort(values.begin(), values.end());
    for (int value : values)
    std::cout << value << std::endl; // 1 2 3 4 5
    std::cin.get();
}
```

加入lambda表达式：

```cpp
std::sort(vec.begin(), vec.end(), [](int a, int b) {
	return a < b; // 如果为真，则a会在b前面
});
```

加内置函数添加头文件functional，使用std::greater函数，则会按照从大到小顺序排列。

```cpp
#include<iostream>
#include<vector>
#include<algorithm>
#include<functional>

int main()
{
    std::vector<int>  values = {3, 5, 1, 4, 2};
    std::sort(values.begin(), values.end(),std::greater<int>());
    for (int value : values)
    std::cout << value << std::endl; // 5 4 3 2 1
    std::cin.get();
}
```

## 66. C++的类型双关（type punning）

> 类型双关只是一个花哨的术语，用来在C++中绕过类型系统。

1. 将同一个内存的东西通过不同type的指针给取出来
   原始方法：（**取地址，换成对应类型的指针，再解引用**）

类型双关常用于不想处理某种类型的复制或转换时。2. 将一个结构体转换成数组进行操作

```
#include <iostream>

struct Entity{
	int x, y;
	void GetPosition() {
		return &x;
	}
};

int main () {
	Entity e;
	int* position = e.GetPosition();
	std::cout << position[1] << position[2] << std::endl;
}
```

类型转换就是说，我要把我拥有的这段聂村，当作不同类型的内存来对待。

## 67. C++的联合体（共用体）

1. `union {}`; 注意结尾由分号。
2. 通常 `union` 是匿名使用的，但是匿名 `union` 不能含有成员函数。
3. 在可以使用**类型双关**的时候，使用 `union`时，可读性更强。
   > 比如定一个 `union` ，包含一个 `int` 和一个`float`。通过类型双关就可以在同一个内存上使用不同的类型来解释这个内存：

```cpp
union {
	int i;
	float f;
};
i = 32;
std::cout << i << std::endl; // 使用int类型来解释
std::cout << f << std::endl; // 使用float类型来解释
```

4.  `union` 的特点是**共用内存**。可以像使用结构体或者类一样使用它们，也可以给它添加静态函数或普通函数、方法等，但是不能使用**虚方法**。

```cpp
#include <iostream>
int main()
{
	struct Union {
	    union { // 通常匿名使用
	        float a;
	        int b;
	    };
	}
	Union u;
    u.a = 2.0f; // 共享内存，a被赋值了一个浮点数，整形b也被赋值了一个浮点数。
    std::cout << u.a << ", " << u.b << std::endl;
// 输出结果：2, 1073741824
// 原因：int b取了组成浮点数的内存，然后把它解释成一个整型（类型双关）
    std::cin.get();
}
```

> 注意！ 匿名联合体和匿名结构体的区别：

```cpp
union {
	int a, b;
}; // 这里的 a,b 是可以直接被使用的
std::cout << a << b << std::endl; // OK

struct {
	int a, b;
}; //a和b不可以直接访问，因为a和b占用不同内存，只能通过实例化对象的名字来访问。
std::cout << a << b << std::endl; // Error
```

**结构体和联合体的重要区别**

1.  _结构体在声明的时候不会分配内存，所以无法被访问，只有在实例化对象的时候才被分配内存_
    _联合体在声明的时候就已经被分配内存，而且成员共享一块内存，所以可以直接被访问。_
2.  _联合体的成员共享同一块内存，所以匿名时也可以直接访问；_
    _结构体的成员分配给不同的内存，要想访问必须有结构体名;_
    **所以，在匿名联合体中声明的匿名结构体也会被分配内存，因为它属于联合体的成员**，但是也不可以直接被访问，因为结构体内部成员被分配在不同的内存上，只能通过结构体名来访问.

```cpp
union {
	struct {
		int a, b;
	}A;
}
A.a = 2; // 这样是可以的！
```

5. 更实用的一个写法：

```cpp
#include <iostream>
struct Vector2
{
    float x, y;
};

struct Vector4
{
    union // 不写名称，作为匿名使用
    {
        struct //第一个Union成员
        {
            float x, y, z, w;
        };
        struct // 第二个Union成员，与第一个成员共享内存
        {
            Vector2 a, b;//a和x，y的内存共享，b和z，w的内存共享
        };
    };
};

void PrintVector2(const Vector2 &vector)
{
    std::cout << vector.x << ", " << vector.y << std::endl;
}

int main()
{
    Vector4 vector = {1.0f, 2.0f, 3.0f, 4.0f};
    PrintVector2(vector.a);
    PrintVector2(vector.b);
    vector.z = 500;
    std::cout << "-----------------------" << std::endl;
    PrintVector2(vector.a);
    PrintVector2(vector.b);
}
//输出：
1，2
3，4
-----------------------
1，2
500，4
```

_当想用多种方法来处理相同的数据(类型双关)时，使用联合体会非常方便_ 可以参考例子
在嵌入式数据通信里面，大部分都是使用联合体，将有实际意义的结构体和字节数组互相转换。

```c
typedef struct
{
    uint8_t head;     //头
    uint8_t cmd;      //功能字
    int16_t x;        //传感器x轴数据
    int16_t y;        //传感器y轴数据
    int16_t z;        //传感器z轴数据
    uint8_t end;      //尾
    uint8_t checkSum; //校验
}Packet_t;

typedef union
{
    uint8_t data[10];
    Packet_t packet;
}Datagram_t;

Datagram_t datagram;
```

如上所示，如果已经通过 datagram.data 接收到数据，想要从数据中提取传感器x轴数据，则只要通过 datagram.packet.x 就可以获取x轴数据。

## 68. C++的虚析构函数

标记为 `virtual` 意味着C++知道可能会有一个方法，在层次结构下的某种重写的方法。

> 如果给基类析构函数设置为虚函数，就会先调用派生类析构函数，然后在层次结构中向上，调用基类析构函数。

1. 如果用基类指针来引用派生类对象，那么基类的析构函数必须时 `virtual`的，否则C++只会调用基类的析构函数，不会调用派生类的析构函数：

```cpp
class Base {
public:
    Base() { std::cout << "Base Constructor\n"; }
    ~Base() { std::cout << "Base Destructor\n"; }
};

class Derived : public Base {
public:
    Derived() { std::cout << "Derived Constructor\n"; }
    ~Derived() { std::cout << "Derived Destructor\n"; }
};
Base *poly = new Derived();
delete poly; // 基类析构函数不加virtual,此处造成内存泄漏。
输出结果：
Base Constructor
Derived Constructor
Base Destructor
没有调用派生类的析构函数
```

2. 继承时，要养成的一个好习惯就是，**基类析构函数中，加上 `virtual`。**

   > 如果没有调用派生类析构函数，当我们在派生类的构造函数中使用堆分配了内存时，不调用派生类析构函数来释放内存，而指向该对象的基类指针已经被删除，堆上的内存无法被删除，就造成了 **内存泄漏**。

3. 在普通方法前标记`virtual` ，那么它就可以被覆写，但是析构函数不一样， **虚析构函数的意思不是覆写析构函数。**
   > 引自知乎文章：
   > **基类中只要定义了虚析构（且只能在基类中定义虚析构，子类析构才是虚析构，如果在二级子类中定义虚析构，编译器不认，且`virtual`失效），在编译器角度来讲，那么由此基类派生出的所有子类地析构均为对基类的虚析构的重写，当多态发生时，用父类引用，引用子类实例时，此时的虚指针保存的子类虚表的地址, 即该函数指针数组中的第一元素永远留给虚析构函数指针。所以当delete 父类引用时，即第一个调用子类虚表中的子类重写的虚析构函数此为第一阶段。然后进入第二阶段：（二阶段纯为内存释放而触发的逐级析构与虚析构就没有半毛钱关系了）而当子类发生析构时，子类内存开始释放，因内存包涵关系，触发父类析构执行，层层向上递进，至到子类所包涵的所有内存释放完成。**

_个人理解就是：_

- 虚析构函数只能定义在基类中，此时所有的子类析构函数都是对基类析构函数的重写；
- 当在基类加了虚析构函数后，不管从那一级开始，当删除父类时，都会从指向的子类的析构函数开始调用。
- 当没有加虚析构函数时，删除一个指向子类的父类指针时，只会是**内存释放而触发的逐级向上析构**，而该父类的子类的析构函数不会被调用；

```cpp
#include <iostream>
class Base {
public:
    Base() { std::cout << "Base Constructor\n"; }
    ~Base() { std::cout << "Base Destructor\n"; }
};

class Derived : public Base {
public:
    Derived() { std::cout << "Derived Constructor\n"; }
    ~Derived() { std::cout << "Derived Destructor\n"; }
};
class Derived1 : public Derived {
public:
    Derived1() { std::cout << "Derived1 Constructor\n"; }
    ~Derived1() { std::cout << "Derived1 Destructor\n"; }
};

int main()
{
    Base* base = new Base();
    delete base;
    std::cout << "-----------------\n";
    Derived* derived = new Derived();
    delete derived;
    std::cout << "-----------------\n";
    Base* poly = new Derived();
    delete poly;
    std::cout << "-----------------\n";
    Derived* poly1 = new Derived1();
    delete poly1;


    std::cin.get();
}
```

输出结果：

```sh
Base Constructor
Base Destructor
-----------------
Base Constructor
Derived Constructor
Derived Destructor
Base Destructor
-----------------
Base Constructor // 只调用了父类
Derived Constructor
Base Destructor
-----------------
Base Constructor
Derived Constructor 只调用了父类，然后逐级向上调用
Derived1 Constructor
Derived Destructor
Base Destructor
```

## 69. C++的类型转换（casting）

> cast分为 `static_cast` 、`dynamic_cast`、`reinterpret_cast` 、`const_cast`

**static_cast**
`static_cast` 用于进行比较“自然” 和低风险的转换，如整型和浮点型、字符型之间的互相转换，不能用于指针类型的强制转换。

> 任何具有明确定义的类型转换，只要不包含底层const，都可以使用 `static_cast`。

```cpp
double dpi = 3.1415926;
int num = static_cast<int>(dPi); // num的值为3
double d = 1.1;
void *p = &d;
double *dp = static_cast<double *>(p);
```

**reinterpret_cast**
`reinterpret_cast` 用于进行各种不同类型的指针之间强制转换。

> 通常为运算对象的位模式提供较低层次上的重新解释。

```cpp
int* ip;
char* pc = reinterpret_cast<char *>(ip);
```

**const_cast**
`const_cast` 用来添加或者移除 `const` 性质。

> 用于改变运算对象的底层 `const`。 常用于有函数重载的上下文中。
> **顶层const** : 表示对象是常量。比如 `int *const P1 = &i` 指针p1本身是一个常量，不能改变p1的值，p1是顶层const.
> **底层const** : 与指针和引用等复合类型部分有关。比如 `const int *p2 = &ci;` 指针所指的对象是一个常量，允许改变p2的值。

**dynamic_cast**
`dynamic_cast` 不检查转换安全性，仅运行时检查，如果不能转换，返回NULL；

## 70. 条件和操作断点

## 71. 现代化C++中的安全

_用于生产环境使用智能指针，用于学习和了解工作积累，使用原始指针_

## 72. C++预编译头文件

> precomolied header(预编译头文件)实际上就是让你抓取一堆头文件，并将它们转换成编译器可以使用的格式，而不必一遍又一遍地读取这些头文件。

1.  作用：
    为了解决一个项目中同一个头文件被反复编译的问题，使得写代码时不需要一遍又一遍的去 `#include`那些常用的头文件，而且能大大提高编译速度。
2.  使用限制：预编译头文件中的内容最好都是**不需要反复更新修改的东西**
    像**C++标准库**，window的api这种不会大改的文件可以放到预编译头文件中，可以节省编译时间。
3.  3.缺点：
    > 预编译头文件的使用会隐藏掉这个cpp文件的依赖。比如用了`#include <vector>`，就清楚的知道这个cpp文件中需要vector的依赖，而如果放到预编译头文件中，就会将该信息隐藏。

## 73. C++的`dynamic_cast`

> `dynamic_cast`是专门用于**沿继承层次结构**进行的强制类型转换，并且 `dynamic_cast`只能用于多态类型。
> 就是说，如果我们有一个类，它是另一个类的子类，我们想要将其转换为父类，或者将父类转化成派生类，这就是`dynamic_cast` 的使用场景.

- 如果将一个子类转化成它的父类，这很简单，因为子类的类型就包含成父类，可以直接进行隐式转换。
- 如果反过来，将一个父类转换成它其中的一个子类，但是这会导致意外的错误。
  在值只能值呢质嫩值呢只能值只能值呢质嫩只能与哟用拥有用于的度多它堕胎多它它台态了类型雷西类新型。

1. 如果转换失败会返回`NULL`，使用时需要保证是多态，即基类里面含有虚函数。
2. dynamic_cast运算符，用于将基类的指针或引用安全地转换成派生类的指针或引用。
   > 支持运行时类型识别(run-time type identification,RTTI)。它存储了运行时类型信息。
   > 适用于以下情况：我们想使用基类对象的指针或引用执行某个派生类操作并且该操作不是虚函数。
3. 使用形式：
   其中，**type必须是一个类类型**，并且通常情况下该类型应该**含有虚函数**。

```cpp
dynamic_cast<type*> (e) //e必须是一个有效的指针
dynamic_cast<type&> (e) //e必须是一个左值， 左值引用
dynamic_cast<type&&> (e) //e不能是左值，右值引用
```

- `dynamic_cast` 一般用于将基类指针转化成派生类指针，会进行运行时类型检测，确保这个基类指向的是该派生类，否则返回NULL；
- 将派生类转化成基类时，不需要使用`dynamic_cast`，因为派生类肯定是基类的，这是安全的，可以使用 `static_cast`来转换；
  以上所有形式，符合三个条件之一，就可以转换成功：
- e是目标类型type的基类，**而且e指向或引用的是目标类型的对象**
- e是目标类型type的派生类，肯定可以转换成功，e本身也是其基类的类型；
- e的类型和目标类型一致；
  代码案例，帮助理解：

```cpp
//代码参考：https://zhuanlan.zhihu.com/p/352420950
#include<iostream>
class Base
{
public:
    virtual void print(){}
};
class Player : public Base
{
};
class Enemy : public Base
{
};
int main()
{
    Player* player = new Player();
    Base* base = new Base();
    Base* actualEnemy = new Enemy();
    Base* actualPlayer = new Player();

    // 旧式转换
    Base* pb1 = player; // 从下往上，是隐式转换，安全
    Player*  bp1 = (Player*)base; // 从上往下，可以用显式转换，危险
    Enemy* pe1 = (Enemy*)player; // 平级转换，可以用显式转换，危险

    // dynamic_cast
    Base* pb2 = dynamic_cast<Base*>(player); // 从下往上，成功转换
    Player* bp2 = dynamic_cast<Player*>(base); // 从上往下，返回NULL
    if(bp2) { } // 可以判断是否转换成功
    Enemy* pe2 = dynamic_cast<Enemy*>(player); // 平级转换，返回NULL
    Player* aep = dynamic_cast<Player*>(actualEnemy); // 平级转换，返回NULL
    Player* app = dynamic_cast<Player*>(actualPlayer); // 虽然是从上往下，但是实际对象是player，所以成功转换
}
```

- `unsigned char` 的范围从 `0` 开始，至少到 `255`
- `unsigned int` 的范围从 `0` 开始，至少到 `65,535`
- `unsigned short` 的范围从 `0` 开始，至少到 `65,535`
- `unsigned long` 的范围从 `0` 开始，至少到 `4,294,967,295`

## 74. C++代码的性能基准测试
> 记住，一定要确保分析的代码在发布时是有意义的，性能基准分析应该确保在发布阶段下有意义。

时间基准测试类`Timer`:
```cpp
class Timer{
public:
	Timer() {
		_startTimePoint = std::chrono::high_resolution_clock::now();
	}
	~Timer() {
		stop();
	}
	
	void stop () {
		// 结束的时间点
		auto endTimePoint = std::chrono::high_resolution_clock::now();
		// 计算到开始时间点度过的时间
		auto start = std::chrono::time_point_cast<std::chrono::microseconds>(_startTimePoint).time_since_epoch().count();
		// 计算到结束时间点度过的时间
		auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimePoint).time_since_epoch().count();
		auto duration = end - start;
		double ms = duration * 0.001;

		std::cout << duration << "us (" << ms << "ms)\n";
	}
private:
	std::chrono::time_point<std::chrono::high_resolution_clock> _startTimePoint;
};

```

使用以上 Timer类测试一下 `shared_ptr` 、`unique_ptr` 和 `new`之间的时间：
```cpp
int main() {
	struct Vector2 {
		float x, y;
	};
	std::cout << "Make Shared\n";
	{
		std::array<std::shared_ptr<Vector2>, 1000> sharedPtrs;
		Timer timer;
		for (int i = 0; i < sharedPtrs.size(); ++i) {
			sharedPtrs[i] = std::make_shared<Vector2>();
		}
	}
	std::cout << "New Shared\n";
	{
		std::array<std::shared_ptr<Vector2>, 1000> sharedPtrs;
		Timer timer;
		for (int i = 0; i < sharedPtrs.size(); ++i) {
			sharedPtrs[i] = std::shared_ptr<Vector2>(new Vector2());
		}
	}
	std::cout << "Make Unique\n";
	{
		std::array<std::unique_ptr<Vector2>, 1000> sharedPtrs;
		Timer timer;
		for (int i = 0; i < sharedPtrs.size(); ++i) {
			sharedPtrs[i] = std::make_unique<Vector2>();
		}
	}


	return 0;
}
```
**结果如下：**
```sh
Make Shared
49us (0.049ms)
New Shared
86us (0.086ms)
Make Unique
33us (0.033ms)
```
>可以看出 `make_shared`和`make_unique`性能高于`new`。


## 75. C++的结构化绑定
>在某些场景中我们可能需要返回多个不同类型的值，以前的方法是可以定义一个结构体，里面包含多个不同类型的值。

C++17之前，除了使用结构体，只能使用 `std::get<返回值序号>(变量名)`和 `std::tie(变量名，变量名，...)`来访问多个类型返回值。
对于这个返回`string`和 `int`l类型的值来说：
```cpp
std::tuple<std::string, int> CreatePerson() {
	return {"Cherno", 24};
}
```
可以使用以下两种方法访问：
1. `std::get<>()`
```cpp
std::tuple<std::string, int> person = CreatePerson();
std::string &name = std::get<0>(person);
int age = std::get<1>(person);
```
2. `std::tie()`
```cpp
std::string name;
int age;
std::tie(name, age) = CreatePerson();
```

C++17版本引入了**结构化绑定**：*直接一行代码搞定*
```cpp
auto[name, age] = CreatePerson();
```

>注意，该方法只在C++17和之后的版本起作用，之前的版本编译器会报错。

## 76. 如何处理OPTIONAL数据？
> OPTIONAL即可能存在，也可能不存在的数据。比如，如果文件是空的，我们应该有办法看到数据是存在还是不存在。 这就需要用到了C++17中的 `std::optional`

TODO

## 82. C++的单例模式
> 单例模式是一种设计模式。那么什么是单例，为什么要用它，什么时候用它？

*单例是一个类的单一实例，也就是说你只打算有那个类或结构体的一个实例。*

**单例的行为就像命名空间，单例类可以像命名空间一样工作。** 
也就是说：**C++中的单例只是一种组织一堆全局变量和静态函数的方式。** 单例就是一个外表为类形式的命名空间。
最基础的单例类：
```cpp
class Singleton {
public:
	// 删除赋值构造函数，防止通过赋值构造另一个实例。
	Singleton(const Singleton &) = delete;
	static Singleton &Get() {
		return _Instance;
	}
	void Func() {}
private:
	Singleton() {}
	static Singleton _Instance;
};

Singleton Singleton::_Instance;
```
访问方式：
```cpp
int main() {
	Singleton &instance = Singleton::Get(); // Ok！
	Singleton instance = Singleton::Get(); // error! 防止构造其他实例，赋值构造已删除。
	Singelton::Get().Func();
	return 0;
}
```
实现一个随机数生成器单例类的例子：
```cpp
#include <iostream>

class Random {
public:
	// 删除赋值构造函数，防止通过赋值构造另一个实例
	Random(const Random&) = delete;
	Random &operator=(const Random &) = delete;
	static Random &Get() {
		static Ramdom instance; // 也可以直接在静态函数中创建一个静态局部变量。而不再使用静态成员变量。
		return instance;
	}
	void Func() {}
	float Float() {
		return _RandomGenerator;
	};
private:
	Random() {}
	// static Random _Instance;
	float _RandomGenerator = 0.5f;
};
 // Random Random::_Instance; // 定义一个实例，以便在静态函数中使用。

int main() {
	float number = Random::Get().Float();
	return 0;
}
```
单例类的一般组成如下：
- 标记为删除的拷贝构造函数；
- 标记为删除的赋值运算符；
- `Get()` 函数用来获取实例；
-  返回成员变量的函数；
- 私有构造函数防止在类外实例化对象；
- 定义一个单例的唯一实例：`Random Random::_Instance;`
主要是单例类同样具有类的一些性质，比如可以返回**类成员变量**的函数。

`float number = Random::Get().Float();` 这样有点麻烦，如何可以实现直接`float number = Random::Float()`?
```cpp
// 将 Float()函数移动到private中，然后在public中定义一个调用该函数的静态函数；
public:
	static float Float() {
		return Get().IFloat();	
	}
private:
	float IFloat() { return _RandomGenerator; }
```
> 这样类内的函数被内联后将不会有任何性能损失。

*如果成员函数在类定义中定义（即在类的声明中直接定义函数体），它被隐式地视为内联函数*
```cpp
static Random& Get() {
	static Random _Instance;
	return _Instance;
}
```

**单例的核心**就是这个 `Get()`函数，只有第一次使用它的时候，创建一个单独实例。因为这个实例是静态局部的，所以只会在第一次调用`Get()`函数时被创建一次，且生命周期和程序生命周期一致。*一旦有了这个实例，就可以写任意数量的非静态函数，并通过Get()函数来调用它们。*

>Tips: 如何想不通过 `Get()`函数，直接`Random::`来调用，可以通过将非静态成员函数变为私有，然后写一个静态函数，在其内部返回非静态函数的调用:)

最后一个问题? 为什么不直接使用命名空间？而要定义一个单例类？
> 1. 类更加好管理，可以延迟初始化，只用实例化时才被创建（静态局部变量；
> 2. 单例类可以将构造函数和成员变量设为私有，控制对实例的访问，增强封装性和安全性。

类的实例化方式有：
1. **直接实例化**：通过构造函数直接创建对象。
2. **动态分配**：使用 `new` 关键字在堆上创建对象。
3. **拷贝构造**：通过拷贝构造函数创建对象。
4. **单例模式**：通过静态方法获取类的唯一实例。
## 83. C++中的小字符串优化
> 小字符串，可能不需要堆分配，可以只分配一小块基于栈的缓冲区，而不是堆分配。这就是静态字符串存储，对于小于一定长度的字符串。
*在编译器中，只有超过一定长度的字符的字符串才会调用`heap malloc`来分配内存，vs2019上小于15个字符的字符串会使用栈的缓冲区。*

如果将一个常量字符串赋值给 `std::string`，当字符串长度超过15个字节时，意味着堆分配。比如：
```cpp
std::string name = "Cherno";
```
而使用`const char* name = "Cherno;"是一个静态字符串，我们不会追加东西，因此分配了一块基于栈的缓冲区。`

使用代码验证一下：
```cpp
#include <iostream>
void *operator new(size_t size) {
	std::cout << "Allocating" << " " << size << " bytes\n";
	return malloc(size);
}
int main() {
	std::string name = "Cherno";// 小于15个字符
	std::string name1 = "Cherno Small Str"; // 16个字符
}
```

Release模式下 `std::string name = "Cherno";` 将只在栈上分配内存，打印结果为空；
```sh

D:\Vs17\Cpp\Cpp_BT\Release\01_Cpp.exe (进程 9696)已退出，返回代码为: 0。
若要在调试停止时自动关闭控制台，请启用“工具”->“选项”->“调试”->“调试停止时自动关闭控制台”。
按任意键关闭此窗口...

```

Release模式下 `std::string name = "Cherno Small Str";` 字符串大于15个，将会在堆上分配内存，调用`new`操作符：
```sh
Allocating 32 bytes

D:\Vs17\Cpp\Cpp_BT\Release\01_Cpp.exe (进程 22248)已退出，返回代码为: 0。
若要在调试停止时自动关闭控制台，请启用“工具”->“选项”->“调试”->“调试停止时自动关闭控制台”。
按任意键关闭此窗口...

```
## 84. 跟踪内存分配的简单方法
> 想要跟踪内存，一个简单的方法就是在 重载`operator new()`函数中添加断点。
```cpp
#include <iostream>
#include <memory>

void *operator new(size_t size) {
	std::cout << "Allocating " << size << " bytes\n";
	return malloc(size);
}

void operator delete(void *memory, size_t size) {
	std::cout << "Freeing " << size << " bytes\n";
	free(memory);
}

struct Object {
	int x, y, z;
};

int main() {
	{
		std::unique_ptr<Object> obj = std::make_unique<Object>();
	}
	std::string name = "Cherno";

	return 0;
}
```
打印结果：
```sh
Allocating 12 bytes
Freeing 12 bytes
Allocating 8 bytes
Freeing 8 bytes
```
更进一步优化,使用 `AllocationMetrics`结构体：
```cpp
#include <iostream>
#include <memory>

struct AllocationMetrics {
	uint32_t TotalAllocated = 0;
	uint32_t TotalFreed = 0;
	uint32_t  CurrentUsage() {
		return TotalAllocated - TotalFreed;
	}
};
static AllocationMetrics s_AllocationMetrices;

void *operator new(size_t size) {
	s_AllocationMetrices.TotalAllocated += size;
	return malloc(size);
}

void operator delete(void *memory, size_t size) {
	s_AllocationMetrices.TotalFreed += size;
	free(memory);
}

struct Object {
	int x, y, z;
};

static void PrintMemoryUsage() {
	std::cout << "Memory Usage: " << s_AllocationMetrices.CurrentUsage() << " bytes\n";
}

int main() {
	PrintMemoryUsage();
	std::string name = "Cherno";
	PrintMemoryUsage();
	{
		std::unique_ptr<Object> obj = std::make_unique<Object>();
		PrintMemoryUsage();
	}
	PrintMemoryUsage();
	return 0;
}
```
打印结果：
```sh
Memory Usage: 0 bytes
Memory Usage: 8 bytes
Memory Usage: 20 bytes
Memory Usage: 8 bytes
```
## 85. 左值和右值

1. 左值

   > 左值有地址和值，**往往长期存在**。左值是由某种存储支持的变量，可以出现在赋值运算符左边或者右边。

2. 左值引用

   > 左值引用仅仅接受左值，除非用了 `const` 兼容（非const的左值引用只接受左值）所以C++常用常量引用。因为**常量引用兼容临时的右值和实际存在的左值变量。**

3. 右值

   > 右值是 **临时量**，无地址（或者说有地址但访问不到，只是一个临时量）没有存储空间短暂存在的值。_可以通过常引用或右值引用来延长右值的生命周期_

4. 右值引用
   > 右值引用不能绑定到左值，形式是：`类型名&&`。

**有名字的右值引用是一个左值**。如 `std:string &&value = "cherno"`
没名字的右值引用是一个右值。如 `std::string("cherno")`

5. 右值引用的优势

   > 如果我们知道传入的是一个临时对象的话，那么我们就不需要担心它们是否活着，是否完整，是否拷贝。我们可以简单地偷它的资源，给到特定的对象，或者其他地方使用它们。因为我们知道它是暂时的，它不会存在很长时间 而如果如上使用const string& str，虽然可以兼容右值，但是却不能从这个字符串中窃取任何东西！因为这个str可能会在很多函数中使用，不可乱修改！（所以才加了const）

> 右值引用作为形参，只接受右值作为实参，因此，右值引用可以用来检测临时变量。
> 【注意！】这里作为形参的右值引用是有变量名 `value`的，因此是一个左值。

6. 给函数传参时的四种情况

```cpp
// 接受左值或右值，仅仅是拷贝传递
void PrintName(std::string name) {
    std::cout << "[rvalue]: " << name << std::endl;
}
// 接受左值或右值，引用传递，但函数内无法修改变量。
void PrintName(const std::string& name) {
    std::cout << "[rvalue or lvalue]: " << name << std::endl;
}
// 只接受右值
void PrintName(std::string&& name) {
    std::cout << "[rvalue]: " << name << std::endl;
}
// 只接受左值，且不能是const
void PrintName(std::string& name) {
    std::cout << "[lvalue]: " << name << std::endl;
}
```

## 86. 持续集成(CI)
>CI通常指在开发期间持续集成代码的过程，本质就是构建自动化和测试，确保代码在所有平台和所有配置下都可以编译。

目的：建立一个C++项目，以便可以再每次提交到github时，自动构建和测试我们的应用。

可以使用类似 `Jenkins`这样的工具来写一个项目包含C++项目源文件和脚本文件，上传到服务器上，每次更改提交并上传代码后，服务器自动启动测试脚本来测试程序。

TODO: 尝试一次自己的自动化项目测试流程。

## 87. C++静态分析
> 静态分析工具就是自动化地code review.

## 88. 参数求值顺序
*argument evaluation order~*
C++标准并没有给大部分运算符规定参数求值顺序，其行为是“**未定义**”的。会根据编译器的不同而变化，完全依赖于C++编译器将代码转换成机器码的实际实现。
比如在vs17 release 模式， C++17之前的版本中实现以下代码：
```cpp
#include <iostream>

void PrintSum(int a, int b) {
	std::cout << a << " + " << b << " = " << (a + b) << std::endl;
}
int main() {
	int value = 0;
	PrintSum(value++, value++); // 0 + 1 = 0 + 1?
	return 0;
}
```

打印结果是：
```sh
0 + 0 = 0
```
说明参数也可以不按顺序传递，而是并行传递的。
而在C++17中执行结果是：
```sh
1 + 0 = 1
```
> 因为**C++17引入一条新规定，后缀表达式必须要其他表达式之前被计算，也就是不能并行计算，只能一个一个计算，但是计算顺序仍是不确定的。**

## 89. C++的移动语义
C++11引入了**右值引用，使得移动语义即允许我们移动对象**。拷贝是重新复制一个对象，创建一个新的堆分配。而移动语义可以直接移动对象，而不需要拷贝。
看下面这个例子，在初始化自定义类型变量时，调用了拷贝构造函数：
```cpp
#include <iostream>

class String {
public:
	String() = default;
	String(const char *string) {
		printf("Created!\n");
		_size = strlen(string);
		_Data = new char[_size];
		memcpy(_Data, string, _size);
	}
	String(const String &other) {
		printf("Copied!\n");
		_size = other._size;
		_Data = new char[_size];
		memcpy(_Data, other._Data, _size);
	}
	~String() {
		printf("Destoryed!\n");
		delete[] _Data; // delete后加上[]，只是为了对每个对象调用析构函数，简单类型直接回收内存即可。
	}
	void Print() {
		for (uint32_t i = 0; i < _size; i++) {
			printf("%c", _Data[i]);
		}
		printf("\n");
	}
private:
	char *_Data;
	uint32_t _size;
};
class Entity {
public:
	Entity(const String &name)
		: _name(name) { // 这里初始化_name会调用拷贝构造函数。

	}
	void printName() {
		_name.Print();
	}
private:
	String _name;
};

int main() {
	Entity entity(String("Cherno")); 
	entity.printName();
	return 0;
}
```
打印输出结果：
```sh
Created!
Copied!
Cherno
```
可见，调用了一次拷贝构造函数。

接下来通过使用移动语义，使其不调用拷贝构造函数：
这就需要写一个移动构造函数：
```cpp
String(String &&other) { // 移动构造函数只接受一个右值
	printf("Moved!\n");
	// 将 other对象的内容“偷过来”。
	_size = other._size;
	_Data = other._Data; // 不需要重新创建堆分配，只是赋值指针。

	// 然后将other变成空
	other._size = 0;
	other._Data = nullptr; // 这样就不会delete同一块内存两次了。
}
// Entity类中也添加一个移动构造函数，以便区分拷贝构造函数和只接受右值来初始化
Entity(Entity &&name)
	 ： _name(name) {
}
```
打印结果可以看到：
```sh
Created!
Copied!
Destoryed!
Cherno
``` 
这里仍然有一个 Copied,  为什么？因为在 `Entity`中移动构造函数的初始化列表中仍然调用了`String` 的拷贝构造函数。因为 `_name(name)`中的`name`是一个**有名字的右值引用，是左值。** 

如何才能使传入的形参是一个右值，从而调用 `String` 的移动构造函数呢？
> 也就是`name`本来是 `String &&name = String("Cherno")`, 是一个左值，在`Entity`初始化列表中将其显式地转化成右值：`_name((String&&)name)`。

```cpp
Entity(Entity &&name)
	 ： _name((String &&)name) {
}
```
打印结果：
```sh
Created!
Moved!
Destoryed!
Cherno
```

**实际中不会使用类型`(String&&)name`来转换，会使用 `std::move(name)`**:
```cpp
Entity(Entity &&name)
	 : _name(std::move(name)) {
}
```

## 90. `std::move` 和移动赋值操作符
> `std::move`的作用将一个对象转化成右值引用，从而启用移动语义，用于移动构造函数中，帮助将一个对象资源移动到另一个对象，而不是复制。

移动赋值操作符的作用是将一个对象的资源移动给另一个对象，而不进行赋值，减少内存的开销。
**移动赋值操作符重载**：
```cpp
String& operator=(String &&other) {
	printf("Moved!\n");
	// 如果自己移动赋值自己则直接返回，不进行任何操作。
	if (this != &other) {
		delete[] _Data; // 使用=的变量已存在，如果不先清除，直接覆盖，会造成内存泄漏。
		_size = other._size;
		_Data = other._Data;
		// 将other清空
		other._size = 0;
		other._Data = nullptr;
	}
	return *this;
}
```
测试移动赋值操作符：
```cpp
int main() {
	String apple = "Apple";
	String dest;
	std::cout << "Apple: ";
	apple.Print();
	std::cout << "Dest: ";
	dest.Print();
	dest = std::move(apple);
	std::cout << "Apple: ";
	apple.Print();
	std::cout << "Dest: ";
	dest.Print();

	std::cin.get();
	return 0;
}
```
打印输出结果：
```sh
Created!
Apple: Apple
Dest:
Moved!
Apple:
Dest: Apple

Destoryed!
Destoryed!
```
可以看到将整个字符数组从对象`apple`转移到了`dest`，而没有做任何复制，没做任何分配或解除分配之类的事情。

**C++三法则：**
- 如果需要析构函数，则一定需要拷贝构造函数和拷贝赋值操作符；
**C++五法则：**
- 增加了两个用来支持移动语义：一定需要移动构造函数和移动赋值运算符。

使用移动赋值运算符和使用构造函数的区别：
```cpp
String apple = "Apple";
String dest = std::move(apple); // 使用移动构造函数
dest = std::move(apple); // 使用赋值移动操作符
```
移动构造函数是作用于正在创建的对象，而赋值移动操作符用于已存在的对象，需要先释放对象的内存再重新赋值，以免造成内存泄漏。

## 91. C++实现Array
> 实现一个固定大小的分配在栈上的Array;

`alloca()`用于在栈上分配内存，函数返回时自动释放，通常用于临时返回区。
`static_assret(条件表达式, "错误信息")`，静态断言是一个在编译时实际评估的断言。
`void *memset(void *s, int c, size_t n)` 用于将一块内存区域设置成想要的值。`s`- 要填充内存块的指针，`c` - 要填充的值，`n` - 要填充的字节数。

```cpp
#pragma once

#include <cstddef> // size_t
#include <stdexcept> // std::out_of_range
#include <iterator> // std::reverse_iterator

// C++标准规定：单下划线+大写字母(_Identifier) 或 双下划线+小写字母(__identifier)的标识符是保留字，理论上用户不得使用。

#if defined(_MSC_VER)
#define _LIBZGCCXX_UNREACHABLE() __assume(0)
#elif defined(__clang__)
#define _LIBZGCCXX_UNREACHABLE() __builtin_unreachable()
#elif defined(__GNUC__)
#define _LIBZGCCXX_UNREACHABLE() __builtin_unreachable()
#else
#define _LIBZGCCXX_UNREACHABLE() \
  do {                            \
  } while (1)
#endif

template <class _Tp, size_t _N>
struct Array {
  using __value_type = _Tp;
  using iterator = _Tp *;
  using const_iterator = _Tp const *;
  using difference_type = std::ptrdiff_t;
  using pointer = _Tp *;
  using const_pointer = _Tp const &;
  using reference = _Tp &;
  using const_reference = _Tp const &;
  using reverse_iterator = std::reverse_iterator<_Tp *>;
  using const_reverse_iterator = std::reverse_iterator<_Tp const *>;

  _Tp _M_elements[_N];
  // size_t 类型可以根据操作系统不同来改变大小，即节省空间又防止空间不够.
  // 加 constexpr 使其编译期求出
  static constexpr size_t size() { return _N; }
  static constexpr size_t max_size() { return _N; }
  // 这里的&引用是为了给数组元素赋值.
  _Tp &operator[](size_t __i) noexcept {
    // 如果传int , 会多发生一步：movzx %eax, %rax ,然后再 lea %rax(%rsp)
    return _M_elements[__i];
  }
  // 要保证 const对象不改变元素的值，这里返回值应该是const.
  // 这里的引用&是为了减少拷贝的开销.
  const _Tp &operator[](size_t __i) const noexcept { return _M_elements[__i]; }

  void swap(Array &__that) noexcept(
      std::is_nothrow_copy_assignable<_Tp>::value) {
    for (size_t __i = 0; __i < _N; __i++) {
      std::swap(_M_elements[__i], __that._M_elements[__i]);
    }
  }

  // noexcept(noexcept(_M_elements[0] = __val)) 表示如果_Tp类型写入无异常,
  // 里层noexcept就返回True, 则整个fill也无异常。
  // 以上写法相当于 noexcpet(std::is_nothrow_copy_assignable<_Tp>)
  void fill(const _Tp &__val) noexcept(
      std::is_nothrow_copy_assignable<_Tp>::value) {
    for (size_t __i = 0; __i < _N; ++__i) {
      _M_elements[__i] = __val;
    }
  }

  _Tp &front() noexcept { return _M_elements[0]; }
  const _Tp &front() const noexcept { return _M_elements[0]; }

  _Tp &back() noexcept { return _M_elements[_N - 1]; }
  const _Tp &back() const noexcept { return _M_elements[_N - 1]; }

  _Tp &at(size_t __i) {
    // 小概率发生的条件表达式，使用 [[unlikely]]修饰，让编译器去优化。
    if (__i >= _N) [[__unlikely__]] {
      throw std::out_of_range("out of range! i = " + std::to_string(__i));
    }
    return _M_elements[__i];
  }
   
  const _Tp &at(size_t __i) const {
    if (__i >= _N) [[__unlikely__]] {
      // throw卸载这里会占用L1i, 指令缓存.[[unlikely]] 会把这些转移到.clod区间。
      throw std::out_of_range("out of range! i = " + std::to_string(__i));
    }
    return _M_elements[__i];
  }
  _Tp const *data() const noexcept { return _M_elements; }
  _Tp *data() noexcept { return _M_elements; }
  _Tp const *cdata() const noexcept { return _M_elements; }

  _Tp *begin() noexcept { return _M_elements; }
  _Tp *end() noexcept { return _M_elements + _N; }

  const _Tp *begin() const noexcept { return _M_elements; }
  const _Tp *end() const noexcept { return _M_elements + _N; }
  const _Tp *cbegin() const noexcept { return _M_elements; }
  const _Tp *cend() const noexcept { return _M_elements + _N; }

  const_reverse_iterator crbegin() const noexcept {
    return const_reverse_iterator(end());
  }
  const_reverse_iterator crend() const noexcept {
    return const_reverse_iterator(begin());
  }

  reverse_iterator rbegin() noexcept {
    return reverse_iterator(end());
  }
  reverse_iterator rend() noexcept {
    return reverse_iterator(begin());
  }

};


// 0长数组偏特化
template <class _Tp>
struct Array<_Tp, 0> {
  using __value_type = _Tp;
  using iterator = _Tp *;
  using const_iterator = _Tp const *;
  using pointer = _Tp *;
  using const_pointer = _Tp const *;
  using reference = _Tp &;
  using const_reference = _Tp const &;
  using reverse_iterator = _Tp *;
  using const_reverse_iterator = _Tp const *;

  _Tp _M_elements[0];
  static constexpr size_t size() { return 0; }
  static constexpr size_t max_size() { return 0; }
  _Tp &operator[](size_t __i) noexcept { _LIBZGCCXX_UNREACHABLE(); }
  const _Tp &operator[](size_t __i) const noexcept { _LIBZGCCXX_UNREACHABLE(); }

  _Tp &front() noexcept { _LIBZGCCXX_UNREACHABLE(); }
  const _Tp &front() const noexcept { _LIBZGCCXX_UNREACHABLE(); }

  _Tp &back() noexcept { _LIBZGCCXX_UNREACHABLE(); }
  const _Tp &back() const noexcept { _LIBZGCCXX_UNREACHABLE(); }

  _Tp &at(size_t __i) {
    throw std::out_of_range("out of range! __i = " + std::to_string(__i));
  }
  const _Tp &at(size_t __i) const {
    throw std::out_of_range("out of range! __i = " + std::to_string(__i));
  }

  _Tp const *data() const noexcept { return nullptr; }
  _Tp *data() noexcept { return nullptr; }
  _Tp const *cdata() const noexcept { return nullptr; }

  _Tp *begin() noexcept { return nullptr; }
  _Tp *end() noexcept { return nullptr; }
  const _Tp *cbegin() const noexcept { return nullptr; }
  const _Tp *cend() const noexcept { return nullptr; }

  const _Tp *begin() const noexcept { return nullptr; }
  const _Tp *end() const noexcept { return nullptr; }

  void fill(const _Tp &) noexcept {}
  void swap(Array &__that) noexcept {}
};

// 实现 a{1, 2, 3} 来创建数组
template <class _Tp, class ..._Ts>
Array(_Tp, _Ts...) -> Array<_Tp, 1 + sizeof...(_Ts)>;
```