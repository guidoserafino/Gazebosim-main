#include <stdio.h>
#include <unistd.h>
#include <iostream>



int main(int argc, char const *argv[]) {
                     
                     int u;
                     std::cout << "Inserisci numero server: " << std::endl;
                     std::cin >> u;
                     for(int i=0; i<u; ++i){
                                             pid_t pid = fork();

 
                                            if(!pid){
                                                        execvp("server/build/main", NULL);
                                                    
                                                     }
                     }
	
	return 0;
 
}




