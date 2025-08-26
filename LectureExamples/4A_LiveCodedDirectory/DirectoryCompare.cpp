// Design a directory of people's names and their Year of birth
//    A person’s name (key) associates with their birth Year (value)
// Compare two design options:
//    Hash table-based
//    Vector-based
// Build your own data structures

// Hash table:
// KEY (name) ---hash---> index ----> lookup entry in a table
// Collision handling: store a table of linked lists; here we just store the values 
// directly and report collisions

// This is as coded in lecture; the "sorted vector" is left as an exercise
// If the directory got to have 1M entries, consider time to add an entry, 
// time to find an entry, and memory utilisation of each structure.


#include <iostream>
#include <string>

struct Person
{
  int Year;
  std::string Name;
};

//--Abstract base--
class CDirectory
{
  public:
    virtual void AddEntry( const char* pName, int Year ) = 0;   // adds an entry to the directory
    virtual int LookupEntry( const char* pName ) = 0;           // looks up an entry
};


//--Hash-based dir---
const int TableSize = 16; // todo: embed inside class
class CDirectoryHash : public CDirectory
{    
  public:
    CDirectoryHash() { std::cout << "todo: init array to zeros" << std::endl; }
    void AddEntry( const char* pName, int Year );
    int LookupEntry( const char* pName );

  private:    
    //--Helper functions---    
    int HashFunction( const char* pName );

    //--Data--    
    int mTableOfData[TableSize];
};


//--Vector-based dir---
const int MaxSize = 16;
class CDirectoryVector : public CDirectory
{    
  public:
    CDirectoryVector() : mSize(0) {}
    void AddEntry( const char* pName, int Year );
    int LookupEntry( const char* pName );

  private:
    int mSize;
    Person mDirectory[MaxSize];
};

//--Vector-based dir---
const int MaxVecSize = 16;
class CDirectorySortedVector : public CDirectory
{    
  public:
    CDirectorySortedVector() : mSize(0) {}
    void AddEntry( const char* pName, int Year ); // here
    int LookupEntry( const char* pName ); // here

  private:
    int mSize;
    Person mDirectory[MaxVecSize];
};



int main()
{
  CDirectory* pMyDirectory = new CDirectoryVector;
//  CDirectory* pMyDirectory = new CDirectoryHash;

  pMyDirectory->AddEntry( "Bob", 2001 );
  pMyDirectory->AddEntry( "James", 1995 );
  pMyDirectory->AddEntry( "Alice", 1800 );
  pMyDirectory->AddEntry( "Darcy", 2004 );
  
  std::cout << "Bob " << pMyDirectory->LookupEntry( "Bob" ) << std::endl;


  delete pMyDirectory;

}

//----------------------------
void CDirectoryVector::AddEntry( const char* pName, int Year )
{
  Person MyPerson;
  MyPerson.Name = pName;
  MyPerson.Year = Year;
  mDirectory[mSize] = MyPerson;
  ++mSize;
}
int CDirectoryVector::LookupEntry( const char* pName ) 
{ 
  int Result = 0;
  for( int i=0; i<mSize; ++i )
  {
    if( mDirectory[i].Name == pName )
    {
      Result = mDirectory[i].Year;
    }
  }

  return Result;
}


//----------------------------
void CDirectoryHash::AddEntry( const char* pName, int Year )
{
  // hash
  int Index = HashFunction( pName );
  std::cout << "Adding " << pName << "'s year at index " << Index << std::endl;
  
  // add entry
  if( mTableOfData[ Index ] != 0 )
    std::cout << "WARNING: HASH COLLISION" << std::endl;

  mTableOfData[ Index ] = Year;
}

int CDirectoryHash::LookupEntry( const char* pName )
{
  // KEY (name) ---hash---> index ----> lookup entry in a table
  int Result = -1;

  // hash
  int Index = HashFunction( pName );
  
  // lookup
  Result = mTableOfData[ Index ];

  return Result;  
}


// returns hash in range 0..TableSize
int CDirectoryHash::HashFunction( const char* pName ) 
{ 
  int Result = 0;
  
  // fast function
  // spreads names out in the table
  for( int i=0; pName[i] != 0; ++i )
  {
    //    Result += pName[i]; // not too random
    Result ^= pName[i];
  }
  
  Result = Result % TableSize;
  
  return Result;  
} 
