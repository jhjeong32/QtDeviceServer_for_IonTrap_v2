## Migration from github.com/jhjeong32snu

A Python-based program for ion trap experiments.
To share experimental equipment across multiple setups, the equipment is controlled by a server computer, allowing multiple clients to access it.

Most equipment controls are designed with multithreading to conduct the necessary operations and run experiments simultaneously. However, for devices like CCDs that require extensive matrix computations and since python cannot properly terminate DLLs, multiprocessing is used instead of multithreading. This allows for terminating processes to stop operations of CCDs.

# QtDeviceServer_v2![QtServer_CCD](https://user-images.githubusercontent.com/63301234/201577976-dabd5510-cda7-4268-b748-679d4d4cebab.png)
