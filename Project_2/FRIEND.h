#ifndef FRIEND_H
#define FRIEND_H

#include <iostream>
#include <string>
using namespace std;

class Friend{
protected:
    string name;
    string message;
    string gift;
    
public:
    Friend(string n);

    void setMessage(string m);

    string getMessage();

    void sayThankYou();

    ~Friend();
};

class BestFriend : public Friend {
private:
    string gift;

public:
    BestFriend(string n, string g);

    void sayThankYou();

    void showGift();

    ~BestFriend();
};

#endif