Build jolt physics engine into a library.

Compile an example C++ program that links to same library- it will initialize the sim and provide an interface to set control inputs, step the sim, and return updated state.

Wrap that C++ with cxx with an application that will call the C++ in a loop, provide any user interface and visualization (e.g. rerun initially).
