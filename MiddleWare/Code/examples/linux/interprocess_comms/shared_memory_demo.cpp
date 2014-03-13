#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
#include <cstdlib>
#include <string>
#include <ostream>

//FUNCTION DECLARATIONS
void newChild ();
void newParent (std::string name);
void child ();
void parent (std::string name);

//CONSTANTS
const char* MEMORY_NAME = "MySharedMemory";

int main(int argc, char *argv[])
{

    if(argc == 1){  //Parent process
        //parent (argv[0]);
        newParent(argv[0]);
    }
    else {
        newChild();
        //child ();
    }
    return 0;
}

void newParent(std::string name)
{
    std::cout << "Starting Parent Process" << std::endl;

    try{
        //A special shared memory where we can construct objects associated with a name.  First remove any old shared memory of the same name, create the shared memory segment and initialize needed resources
        boost::interprocess::shared_memory_object::remove(MEMORY_NAME);
        boost::interprocess::managed_shared_memory segment
            //create segment name segment size
            (boost::interprocess::create_only, MEMORY_NAME, 65536);

        std::string *instance = segment.construct<std::string> 
            ("MyString") //name of the object  
            ("johnboyd's string"); //ctor first argument

        //Launch child process
        std::string s (name); s += " child ";
        std::cout << "Spawning child process" << std::endl;
        std::system(s.c_str());
    }
    catch(...){
        boost::interprocess::shared_memory_object::remove(MEMORY_NAME);
        throw;
    }
    boost::interprocess::shared_memory_object::remove(MEMORY_NAME);
}

void newChild()
{
    std::cout << "Running child process" << std::endl;
    try{
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, MEMORY_NAME);
        std::cout << "Accessing shared segment" << std::endl;
        std::pair<std::string*, std::size_t> rcvd;
        
        //Find the string
        rcvd = segment.find<std::string> ("MyString");

        std::cout << "Found std::string: " << *rcvd.first  << std::endl;

        segment.destroy<std::string>("MyString");
    }
    catch(...){
        std::cout << "Caught exception" << std::endl;
        boost::interprocess::shared_memory_object::remove(MEMORY_NAME);
        throw;
    }
    boost::interprocess::shared_memory_object::remove(MEMORY_NAME);
}
    
void parent (std::string name) 
{
    std::cout << "Starting Parent Process" << std::endl;
    //Remove shared memory on construction and destruction
    struct shm_remove
    {
        shm_remove() { boost::interprocess::shared_memory_object::remove(MEMORY_NAME); }
        ~shm_remove(){ boost::interprocess::shared_memory_object::remove(MEMORY_NAME); }
    } remover;

    //Create a shared memory object.
    boost::interprocess::shared_memory_object shm (boost::interprocess::create_only, MEMORY_NAME, boost::interprocess::read_write);

    //Set size
    shm.truncate(1000);

    //Map the whole shared memory in this process
    boost::interprocess::mapped_region region(shm, boost::interprocess::read_write);

    //Write all the memory to 1
    std::memset(region.get_address(), 1, region.get_size());

    //Launch child process
    std::string s (name); s += " child ";
    std::cout << "Spawning child process" << std::endl;
    std::system(s.c_str());
}

void child ()
{
    std::cout << "Running Child process" << std::endl;
    //Open already created shared memory object.
    boost::interprocess::shared_memory_object shm (boost::interprocess::open_only, MEMORY_NAME, boost::interprocess::read_only);

    //Map the whole shared memory in this process
    boost::interprocess::mapped_region region(shm, boost::interprocess::read_only);

    //Check that memory was initialized to 1
    char *mem = static_cast<char*>(region.get_address());
    for(std::size_t i = 0; i < region.get_size(); ++i)
        if(*mem++ != 1)
            std::cout << "Got correctly initialized memory" << std::endl;

}
