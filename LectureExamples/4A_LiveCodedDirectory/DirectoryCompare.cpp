// Design a directory of people and their year of birth
//    A person’s name (key) associates with their birth year (value)
// Compare two design options:
//    Hash table-based
//    Vector-based
// Build your own data structures

#include <iostream>
#include <string>


class CDirectory
{
  public:
    //---Methods---------------------------------------------------------------
    virtual void AddEntry( const char* pName, int year ) = 0;   // adds an entry to the directory
    virtual int LookupEntry( const char* pName ) = 0;           // looks up an entry
  
  private:
};


int main()
{

}

