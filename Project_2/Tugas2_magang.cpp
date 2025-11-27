#include "FRIEND.h"
#include <iostream>
#include <string>
using namespace std;

Friend::Friend(string n) {
    name = n;
    message = "Thank you for being a great friend!";
}

void Friend::setMessage(string m) {
    message = m;
}

string Friend::getMessage() {
    return message;
}

void Friend::sayThankYou() {
    cout << name << " says: " << message << endl;
}

Friend::~Friend() {}

BestFriend::BestFriend(string n, string g) : Friend(n) {
    gift = g;
}

void BestFriend::sayThankYou() {
    cout << name << " says: " << message << ", I got you a " << gift << "!" << endl;
}

void BestFriend::showGift() {
    cout << name << "'s special gift is: " << gift << endl;
}

BestFriend::~BestFriend() {
}

int main() {
    Friend f1("Afif");
    f1.sayThankYou();

    cout << endl;

    Friend f2("Ruben");
    f2.setMessage("Thanks for carrying me");
    f2.sayThankYou();

    cout << endl;
    
    BestFriend bf1("Fayyadh", "Gaming Chair");
    bf1.sayThankYou();

    cout << endl;

    BestFriend bf2("Abel", "Gaming Laptop");
    bf2.setMessage("Thanks for coaching me");
    bf2.sayThankYou();
    bf2.showGift();

    cout << endl;

    Friend* f3 = &bf1;
    f3->sayThankYou();
    
    return 0;
}
