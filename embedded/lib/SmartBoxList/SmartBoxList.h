#ifndef SMARTBOXLIST_H
#define SMARTBOXLIST_H

#include <Arduino.h>
#include <SmartBoxListConfiguration.h>

class SmartBoxList // stores valuable SmartBoxes
{
public:
  SmartBoxList();
  bool push(String &hostname);     // pushes String to list if list not full
  bool pop(String &hostname);      // pops String from list
  bool isInList(String &hostname); // checks if String is in list
  int getSize();                   // gets size of list

private:
  String hostnames[MAX_SMARTBOXES_TOLISTENTO];
  int iterator;
};

#endif
