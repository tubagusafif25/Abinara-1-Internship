Object-Oriented Friendship Model
Objective
Develop a C++ application that demonstrates core Object-Oriented Programming (OOP) principles by modeling social relationships. The project simulates the hierarchy between a standard "Friend" and a "Best Friend" to showcase inheritance, encapsulation, and method overriding.

Project Structure
cpp-oop-friendship/
├── FRIEND.h       # Header file containing class definitions & hierarchy
└── main.cpp       # Implementation logic and driver code
Components
Component 1: The Base Class (Friend)
Defined a fundamental class to handle basic interactions:

Encapsulation: Uses protected attributes for name and message to ensure data security while allowing access to derived classes.

Core Methods: Implements setMessage() and getMessage() for dynamic interaction.

Behavior: Defines a standard sayThankYou() method to output a gratitude message.

Component 2: The Derived Class (BestFriend)
Created a specialized class that inherits from Friend to extend functionality:

Inheritance: Automatically acquires properties from the Friend class.

Extension: Adds a unique private attribute gift specific to best friends.

Method Overriding: Redefines the sayThankYou() method to include the gift in the output, demonstrating how derived classes can modify parent behavior.

New Functionality: Introduces showGift() to exclusively handle the new attribute.

Technical Concepts Demonstrated
Inheritance: Implementing an "Is-A" relationship where BestFriend is a type of Friend.

Encapsulation: Managing visibility scope (private vs protected vs public).

Method Overriding: Customizing inherited functions to perform specific tasks.

Pointer Logic & Static Binding: Demonstrating how a base class pointer (Friend*) interacts with a derived class object (BestFriend), specifically showcasing static binding behavior in C++.

How to Run
To compile and execute this project using a standard C++ compiler:

Bash

# Compile the program
g++ main.cpp -o friend_app

# Run the executable
./friend_app
Expected Output
Plaintext

Afif says: Thank you for being a great friend!

Ruben says: Thanks for carrying me

Fayyadh says: Thank you for being a great friend!, I got you a Gaming Chair!

Abel says: Thanks for coaching me, I got you a Gaming Laptop!
Abel's special gift is: Gaming Laptop

Fayyadh says: Thank you for being a great friend!

You maybe wondering why Fayyadh doesn't mention the gaming chair in the last line?

Because this demonstrates Static Binding. Even though the object is a 'BestFriend', the pointer is type 'Friend'. Because I didn't use the virtual keyword, C++ decides which function to call at compile-time based on the pointer type, not the object type."
