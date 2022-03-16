#include <stdio.h>
#include <unistd.h>
#include <iostream>



int main(int argc, char const *argv[]) {
                     
                     int u;
                     std::cout << "Inserisci numero droni: " << std::endl;
                     std::cin >> u;
                     for(int i=0; i<u; ++i){
                                             pid_t pid = fork();

 
                                            if(!pid){
                                                        execvp("client/build/main", NULL);
                                                    
                                                     }
                     }
	
	return 0;
 
}




