#include <SmartBoxList.h>

SmartBoxList::SmartBoxList()
{
    iterator = 0;
}

bool SmartBoxList::push(String &hostname)
{
    if (iterator < MAX_SMARTBOXES_TOLISTENTO)
    {
        iterator++ hostnames[iterator] = hostname;
        return true;
    }
    else
        return false;
}

bool SmartBoxList::pop(String &hostname)
{
    if (isInList(hostname))
    {
        int index;
        for (int i = 0; i < MAX_SMARTBOXES_TOLISTENTO; i++) // find index of searched hostname
        {
            if (hostnames[i] == hostname)
                index = i;
        }
        for (int i = index; i < MAX_SMARTBOXES_TOLISTENTO - 1; i++) // copy all strings
        {
            hostnames[i] = hostnames[i + 1];
        }
        hostnames[MAX_SMARTBOXES_TOLISTENTO - 1] = NULL;
        iterator--;
    }
    else
        return false;
}

bool SmartBoxList::isInList(String &hostname)
{
    bool returns = false;
    for (int i = 0; i < MAX_SMARTBOXES_TOLISTENTO; i++)
    {
        if (hostnames[i] == hostname)
            returns = true;
    }
    return returns;
}


int SmartBoxList::getSize()
{
    return iterator;
}
