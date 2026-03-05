// Message sending implementation between field and remote control computers
#include "field_remote.h"
#include <math.h>   // for sin/cos
#include <stdio.h>  // for sprintf
#include <iostream> // for cout
#include <unistd.h> // for sleep, getpid
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <arpa/inet.h>
#include <netinet/tcp.h> // for TCP_NODELAY & SOL_TCP
#include <signal.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <openssl/evp.h>
#include <openssl/aes.h>




// begin AES
static char _key_data[]="u348c/;alDFd0lasd.j809d;lak908lTMkaf549s.,cvHK";
static int _encryption_initialized = 0;
static EVP_CIPHER_CTX* _en_evp_cipher_ctx;
static EVP_CIPHER_CTX* _de_evp_cipher_ctx;

/**
  AES encryption/decryption demo program using OpenSSL EVP apis
  gcc -Wall openssl_aes.c -lcrypto

  this is public domain code. 

  Saju Pillai (saju.pillai@gmail.com)
  modified by Kevin
**/


/**
 * Create a 256 bit key and IV using the supplied key_data. salt can be added for taste.
 * Fills in the encryption and decryption ctx objects and returns 0 on success
 **/
static int aes_init(unsigned char *key_data, int key_data_len, unsigned char *salt, EVP_CIPHER_CTX *e_ctx, 
             EVP_CIPHER_CTX *d_ctx)
{
  int i, nrounds = 5;
  static unsigned char key[32], iv[32];

  /*
   * Gen key & IV for AES 256 CBC mode. A SHA1 digest is used to hash the supplied key material.
   * nrounds is the number of times the we hash the material. More rounds are more secure but
   * slower.
   */
  i = EVP_BytesToKey(EVP_aes_256_cbc(), EVP_sha1(), salt, key_data, key_data_len, nrounds, key, iv);
  if (i != 32) {
    printf("Key size is %d bits - should be 256 bits\n", i);
    return 0;
  }

  EVP_CIPHER_CTX_init(e_ctx);
  EVP_EncryptInit_ex(e_ctx, EVP_aes_256_cbc(), NULL, key, iv);
  EVP_CIPHER_CTX_init(d_ctx);
  EVP_DecryptInit_ex(d_ctx, EVP_aes_256_cbc(), NULL, key, iv);

  return 1;
}

static int init_encryption()
{
    int key_data_len = strlen(_key_data);
    static unsigned int salt[] = {12345, 54321};
    _en_evp_cipher_ctx = EVP_CIPHER_CTX_new();
    _de_evp_cipher_ctx = EVP_CIPHER_CTX_new();
    return aes_init((unsigned char*)_key_data, key_data_len, (unsigned char *)&salt, _en_evp_cipher_ctx, _de_evp_cipher_ctx);
}


/*
 * Encrypt *len bytes of data
 * All data going in & out is considered binary (unsigned char[])
 */
static void aes_encrypt(EVP_CIPHER_CTX *e, unsigned char *plaintext, int *len, unsigned char *ciphertext)
{
  /* max ciphertext len for a n bytes of plaintext is n + AES_BLOCK_SIZE -1 bytes */
  int c_len = *len + AES_BLOCK_SIZE, f_len = 0;

  /* allows reusing of 'e' for multiple encryption cycles */
  EVP_EncryptInit_ex(e, NULL, NULL, NULL, NULL);

  /* update ciphertext, c_len is filled with the length of ciphertext generated,
    *len is the size of plaintext in bytes */
  EVP_EncryptUpdate(e, ciphertext, &c_len, plaintext, *len);

  /* update ciphertext with the final remaining bytes */
  EVP_EncryptFinal_ex(e, ciphertext+c_len, &f_len);

  *len = c_len + f_len;
}

/*
 * Decrypt *len bytes of ciphertext
 */
static void aes_decrypt(EVP_CIPHER_CTX *e, unsigned char *ciphertext, int *len, unsigned char* plaintext)
{
  /* plaintext will always be equal to or lesser than length of ciphertext*/
  int p_len = *len, f_len = 0;
  
  EVP_DecryptInit_ex(e, NULL, NULL, NULL, NULL);
  EVP_DecryptUpdate(e, plaintext, &p_len, ciphertext, *len);
  EVP_DecryptFinal_ex(e, plaintext+p_len, &f_len);

  *len = p_len + f_len;
}


// end AES





static const unsigned short G_START_BYTES = 0x7adb;

// Function to send UDP message, initializes connection every call
static int send_UDP_no_msgID(LOW_LEVEL_MSG *msg, const char *dest, int port)
{
    msg->startBytes = G_START_BYTES;
    // calculate the message size - add our header size
    int msgSize = LOW_LEVEL_MSG_HEADER_SIZE + msg->bytesOfData;
    // fixme - check message size?

    // encrypt the message here
    static LOW_LEVEL_MSG encrypted_msg;
    if (!_encryption_initialized) {
	_encryption_initialized = init_encryption();
    }
    aes_encrypt(_en_evp_cipher_ctx, (unsigned char*)msg,  &msgSize, (unsigned char*)&encrypted_msg);


    
    struct hostent *h;
    h = gethostbyname(dest);
    if (h == NULL) {
	std::cout << "send_UDP couldn't find host " << dest << std::endl;
	return 0;
    }

    struct sockaddr_in addr;
    bzero(&addr, sizeof(sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    memcpy(&addr.sin_addr, h->h_addr_list[0], h->h_length);
    //addr.sin_addr.s_addr = inet_addr(dest);

    int sockfd;
    sockfd = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        // Print or send error
        std::cout << "send_UDP - Couldn't create socket" << std::endl;
	return 0;
    }


    int returnVal = sendto(sockfd, (char*)&encrypted_msg, msgSize, 0, (struct sockaddr*)&addr, sizeof(addr));
    if (returnVal < 0) {
        // Print or send error
        std::cout << "send_UDP - Problem sending data" << std::endl;
	std::cout << msgSize << " " << returnVal << " " << sockfd << std::endl;
    }
    close(sockfd);
    return msgSize;
}

// Function to send UDP message, initializes connection every call
int send_UDP(LOW_LEVEL_MSG *msg, const char *dest, int port)
{
    // Set the message count
    static unsigned short msgCnt = 0;
    msgCnt++;
    msg->msgID = msgCnt;
    return send_UDP_no_msgID(msg, dest, port);
}





int init_recv_UDP(int port)
{
    int sockfd = -12;
    sockfd = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
	std::cout << "init_recv_UDP socket failure " << sockfd << std::endl;
	return(-1);
    }

    struct sockaddr_in addr;
    bzero(&addr, sizeof(sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    int val = 1;
    int val2 = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(int));

    // Increase the receive buffer size
    // The default is 212992 bytes - about 6mS at 300Mbit/s
    val = 16*1024*1024;
    val2 = setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &val, sizeof(int));
    if (val2 == -1) {
	printf("Increasing receive buffer size failed - some more incoming UDP packets may be lost\n");
	printf("Fix by sudo sysctl -w net.core.rmem_max=16777216 or\n");
	printf("Add net.core.rmem_max=16777216 to sysctl.conf\n");
    }
    else {
	printf("Increasing receive buffer success %d %d\n", val, val2);
    }

    if (bind(sockfd, (struct sockaddr*) &addr, sizeof(addr)) != 0) {
	// give some error
	std::cout << "init_recv_UDP- some socket error in bind " << val2 << std::endl;
	return(-1);
    }
    std::cout << "init_recv_UDP success " << sockfd << std::endl;
    return sockfd;
}


// Function to receive UDP message, call init_recv_UDP first
// Return value of 0 - no message received, *msg is unchanged
// Return value of 1 - message received, *msg contains new message
// Return value of -1 - error
int recv_UDP(LOW_LEVEL_MSG *msg, int sockfd)
{
    // Check that msg is not NULL
    if (msg == 0) {
        std::cout << "recv_UDP - msg is NULL" << std::endl;
	return(-1);
    }


    static char buffer[6*sizeof(LOW_LEVEL_MSG)];
    static unsigned char encrypted_buffer[6*sizeof(LOW_LEVEL_MSG)];
    struct sockaddr_in addr;
    int bytes;
    int addr_len = sizeof(addr);

    fd_set fdSet1;
    timeval timeout1; //us
    int selectReturnVal;
    timeout1.tv_sec = timeout1.tv_usec = 0;
  
    FD_ZERO(&fdSet1);
    FD_SET(sockfd, &fdSet1);
    selectReturnVal = select(sockfd+1, &fdSet1, NULL, NULL, &timeout1);
    if (selectReturnVal < 0) {
        std::cout << "recv_UDP An error occurred in the timeout " << errno << std::endl;
	return -1;
    }

    else if (!FD_ISSET(sockfd, &fdSet1)) {
        // cout << "A timeout occurred " << endl;
        return 0;
    }

    bytes = recvfrom(sockfd, encrypted_buffer, sizeof(encrypted_buffer), 0, (struct sockaddr*) &addr, (socklen_t*)&addr_len);
    
    //printf("msg2 from %s:%d (%d bytes)\n", inet_ntoa(addr.sin_addr),
    //	 ntohs(addr.sin_port), bytes);


    if (bytes < LOW_LEVEL_MSG_HEADER_SIZE) {
        std::cout << "recv_UDP -Message size not correct " << bytes << std::endl;
        std::cerr << "recv_UDP -Message size not correct " << bytes << std::endl;
	return -1;
    }

    // decrypt message
    if (!_encryption_initialized) {
	_encryption_initialized = init_encryption();
    }
    aes_decrypt(_de_evp_cipher_ctx, encrypted_buffer, &bytes, (unsigned char*)buffer);


    // Check for correct startBytes;
    if ((*((unsigned short*)&buffer[0])) != G_START_BYTES) {
	printf("recv_UDP - G_START_BYTES mismatch %d %x %x %x\n", bytes, (*((unsigned short*)&buffer[0])), buffer[0], buffer[1]);
	return 0;
    }


    int bytesOfData = *((unsigned int*)&buffer[4]);
    if ((LOW_LEVEL_MSG_HEADER_SIZE+bytesOfData) !=  bytes) {
        std::cout << "recv_UDP - Message size not correct " << bytes << " " << bytesOfData << std::endl;
	if (bytes == 0) {
	    return -1;
	}
	else {
	    // It is possible to receive two messages at once and get to this point
	    // Both messages will be lost.
	    return 0;
	}
    }

    memcpy(msg, buffer, LOW_LEVEL_MSG_HEADER_SIZE+bytesOfData);//sizeof(*msg));
    return 1;
}

