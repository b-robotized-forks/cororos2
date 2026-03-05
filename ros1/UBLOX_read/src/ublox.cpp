#include "UBLOX/ublox.h"
#include <typeinfo>
#include <openssl/evp.h>
#include <openssl/aes.h>
#include <field_remote.h>

#define DBG(...) fprintf(stderr, __VA_ARGS__)


// By default use a fixed position set in the receiver
// Alternate - allow survey in by setting DO_SURVEY_IN to 1
static int DO_SURVEY_IN = 0;


// At the default 5Hz rate a base sends out about 40 messages per second over UDP
// They are in groups of 4 sent within ~1mS of each other
// Unencrypted they are of size 104, 193, 218, 218  and 142, 70, 70, 67
// Encrypted they are of size 106, 282, 250, 250 and 154, 74, 74, 74



// begin AES
static char _key_data[]="v925W/;a3DFd0lAsd.j8q9d;lar908lTMkaf367s.,RvHd";
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


static void encrypt(uint8_t* buf, size_t size, const uint8_t** encrypted_buf, size_t* encrypted_size)
{
    static unsigned char msg_out[13000];
    int msg_out_size = size;
    if (!_encryption_initialized) {
	_encryption_initialized = init_encryption();
    }
    aes_encrypt(_en_evp_cipher_ctx, (unsigned char*)buf,  &msg_out_size, (unsigned char*)msg_out);
    *encrypted_size = msg_out_size;
    *encrypted_buf = msg_out;
}

// end AES










namespace ublox
{

UBLOX::UBLOX(const std::string& port, int message_rate) :
    serial_(port, 460800),
    //115200
    ubx_(serial_)
{
    type_ = NONE;

    auto cb = [this](const uint8_t* buffer, size_t size)
    {
        this->serial_read_cb(buffer, size);
    };
    serial_.register_receive_callback(cb);
    serial_.init();
      
    // configure the parsers/Enable Messages

    ubx_.set_nav_rate(message_rate);
    //configuring SVIN messages is done in config_base_stationary()

    checkSoftware();

    auto eph_cb = [this](uint8_t cls, uint8_t type, const ublox::UBX_message_t& in_msg, uint8_t f9pID=0)
    {
      this->nav_.convertUBX(in_msg.RXM_SFRBX);
    };
    ubx_.registerCallback(ublox::CLASS_RXM, ublox::RXM_SFRBX, eph_cb);

}

bool UBLOX::checkSoftware()
{
    MON_VER_DBG_t mon_ver = ubx_.getVersion();
    if(mon_ver.got_mon)
    {
        MON_VER_t version = mon_ver.mon_ver;
    
        std::string swVersion;
        for(uint8_t i=0; i<30 && version.swVersion[i]!='\0'; i++)
        {
            swVersion.push_back(version.swVersion[i]);
        }

        DBG("Software: %s\n", swVersion.c_str());
        if(swVersion!="EXT CORE 1.00 (61b2dd)")
        {
            DBG("NEEDS FIRMWARE UPDATE\n");
            return false;  
        }
        else
        {
            DBG("FIRMWARE IS CURRENT\n");
            return true;
        }
    }
    else
    {
        return false;
    }
}

void UBLOX::config_gnss(GNSS_CONSTELLATION_t constellation)
{
    // DBG("config_gnss\n");
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, constellation.gps_enable, CFG_VALSET_t::SIGNAL_GPS);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_GPS_L1);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_GPS_L2);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, constellation.galileo_enable, CFG_VALSET_t::SIGNAL_GAL);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_GAL_E1);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_GAL_E5B);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, constellation.beidou_enable, CFG_VALSET_t::SIGNAL_BDS);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_BDS_B1);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_BDS_B2);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, constellation.glonas_enable, CFG_VALSET_t::SIGNAL_GLO);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_GLO_L1);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_GLO_L2);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_QZSS);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_QZSS_L1CA);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::SIGNAL_QZSS_L2C);
}


void UBLOX::config_f9p(uint8_t dynamic_model) //See ubx_defs.h for more information
{
    // DBG("config_f9p\n");
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, dynamic_model, CFG_VALSET_t::DYNMODEL); //Dynamic platform model
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::USB_INPROT_NMEA); //Flag to indicate if NMEA should be an input protocol on USB
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::USB_OUTPROT_NMEA); //Flag to indicate if NMEA should be an output protocol on USB
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_RXM_RTCM);

    bool poll = true;
    if(poll == true)
        poll_value();
}


void UBLOX::config_base(std::string base_type, int gps, int glonas, int beidou,
                  int galileo, int surveytime, int surveyacc)
{
    // DBG("config_base\n");
    // std::cerr<<"Configuring Base\n";
    //Choose to configure as moving/mobile base or stationary
    //bool mobile = false;
    if(base_type == "moving")   //Moving base
    {
        config_base_moving(1, gps, glonas, beidou, galileo);
        config_base_stationary(0, gps, glonas, beidou, galileo, surveytime, surveyacc);
        // std::cerr<<"Moving Base\n";
    }
    else if(base_type == "stationary")  //Stationary base
    {
        config_base_moving(0, gps, glonas, beidou, galileo);
        config_base_stationary(1, gps, glonas, beidou, galileo, surveytime, surveyacc);
        // std::cerr<<"Stationary Base\n";
    }
    else    //Error thrown when type of base is unrecognized
    {
        throw std::runtime_error("Failed to initialize base as moving or stationary");
    }
}

void UBLOX::config_base_stationary(int on_off, int gps, int glonas, int beidou,
                  int galileo, int surveytime, int surveyacc)
{
    // DBG("config_base_stationary\n");
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1005USB);
    if (DO_SURVEY_IN == 1) {
	ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::MSGOUT_SVIN);
	ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::TMODE_MODE);
	// Survey in accuracy limit
	ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, surveyacc*on_off, CFG_VALSET_t::TMODE_SVIN_ACC_LIMIT);
	// Survey in time limit
	ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, surveytime*on_off, CFG_VALSET_t::TMODE_SVIN_MIN_DUR);
    }


    /*
    // If some flag use stationary mode with fixed
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1005USB);
    //ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::MSGOUT_SVIN);
    //ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 2*on_off, CFG_VALSET_t::TMODE_MODE);
    //ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::TMODE_POS_TYPE);

    // Fixed lat/lon/height
    double lat = 34.175936;
    double lon = -118.939002;
    double alt = 220.5; // meters
    //ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, lat * 1e7, CFG_VALSET_t::TMODE_LAT);
    //ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, lon * 1e7, CFG_VALSET_t::TMODE_LON);
    //ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, alt * 1e2, CFG_VALSET_t::TMODE_ALT);

    // Estimated accuracy of fixed position - 50 indicates 5cm
    //ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 50, CFG_VALSET_t::TMODE_FIXED_POS_ACC);
    */
}


void UBLOX::config_base_moving(int on_off, int gps, int glonas, int beidou,
                  int galileo)
{
    // DBG("config_base_moving\n");
    // These values control whether RTK corrections are calculated for the
    // following constellations
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_4072_0USB);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_4072_1USB);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_1077USB);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_1087USB);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_1097USB);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_1127USB);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_1230USB);
}

/**
 * Configures rover settings on F9P
 */
void UBLOX::config_rover()
{
    // DBG("config_rover\n");
    // configure the parsers/Enable Messages
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_PVT);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_RELPOSNED);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_POSECEF);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_VELECEF);
    // ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::MSGOUT_RAWX);
    // ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::MSGOUT_SFRBX);
    //configuring SVIN messages is done in config_base_stationary()
}

/**
 * Polls configuration values on F9P
 */
void UBLOX::poll_value() //See ubx_defs.h for more information
{
    // DBG("poll_value\n");
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1005USB); //CFG-MSGOUT-RTCM_3X_TYPE1005_USB -- Stationary RTK Reference Station ARP
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1230USB); //CFG-MSGOUT-RTCM_3X_TYPE1230_USB -- Glonass L1 and L2 Code-Phase Biases
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1097USB); //CFG-MSGOUT-RTCM_3X_TYPE1097_USB __ Galileo MSM 7 (high precision)
}

/**
 * Not in use anywhere at the moment
 */
void UBLOX::readFile(const std::string& filename)
{
    std::ifstream file(filename,  std::ifstream::binary);
    file.seekg(0, file.end);
    uint32_t len = file.tellg();
    file.seekg (0, file.beg);
    char* buffer = new char [len];
    file.read(buffer, len);
    int idx = 0;
    while(idx < len)
    {
        int chunk_size = ((len - idx) >= 14)?14:len-idx;
        serial_read_cb(((const uint8_t*)buffer)+idx, chunk_size);
        idx = idx+chunk_size;
        usleep(1000);
    }
    // serial_read_cb((const uint8_t*)buffer, len);
}

/**
 * @brief Writes a log file of activity from this class
 * 
 * @param filename Log filename (and path)
 */
void UBLOX::initLogFile(const std::string& filename)
{
    if (log_file_.is_open())
        log_file_.close();

    log_file_.open(filename);
}

void UBLOX::initRover(std::string local_host, uint16_t local_port,
                      std::string remote_host, uint16_t remote_port, uint8_t dynamic_model)
{
    GNSS_CONSTELLATION_t constellation;
    constellation.gps_enable=1;
    constellation.glonas_enable=1;
    constellation.beidou_enable=1;
    constellation.galileo_enable=1;
    initRover(local_host, local_port, remote_host, remote_port, constellation, dynamic_model);
}


void UBLOX::initRover(std::string local_host, uint16_t local_port,
                      std::string remote_host, uint16_t remote_port, GNSS_CONSTELLATION_t constellation, uint8_t dynamic_model)
{
    // std::cerr << "initRover \n";
    type_ = ROVER;

    assert(udp_ == nullptr);
    // Connect the rtcm_cb callback to forward data to the UBX serial port
    rtcm_.registerCallback([this](uint8_t* buf, size_t size)
    {
        this->rtcm_complete_cb(buf, size);
    });

    // hook up UDP to listen
    /// TODO: configure ports and IP from cli
    udp_ = new async_comm::UDP(local_host, local_port, remote_host, remote_port);
    udp_->register_receive_callback([this](const uint8_t* buf, size_t size)
    {
        this->udp_read_cb(buf, size);
    });

    if (!udp_->init())
        throw std::runtime_error("Failed to initialize Rover receive UDP");

    config_gnss(constellation);
    config_ubx_msgs(1);
    config_rtcm_msgs(0, 0, 0, 0, constellation);
    // config_rover();
    config_f9p(dynamic_model);
    std::cerr<<"Initialized Rover\n";
}


// Sends the UDP message to each of the robot control stations
// to be forwarded to the rovers (robots)
void send5(const uint8_t *buf, size_t size)
{
    static LOW_LEVEL_MSG msg;
    static unsigned short msgID = 0;
    msgID++;
    msg.msgID = msgID;
    msg.msgType = MSG_TYPE2_RTCM;
    msg.bytesOfData = size;
    for (unsigned int i = 0; i < msg.bytesOfData; i++) {
	msg.data[i] = buf[i];
    }
    
    send_UDP(&msg, "192.168.3.50", 13014);
    send_UDP(&msg, "192.168.3.49", 13014);
    send_UDP(&msg, "192.168.3.48", 13014);
    send_UDP(&msg, "192.168.3.47", 13014);
    send_UDP(&msg, "192.168.3.46", 13014);
}



void UBLOX::initBase(std::string local_host[], uint16_t local_port[],
                std::string remote_host[], uint16_t remote_port[],
                std::string base_type, int rover_quantity, 
                GNSS_CONSTELLATION_t constellation, int surveytime,
                int surveyacc, uint8_t dynamic_model)
{
    // std::cerr << "initBase \n";
    type_ = BASE;

    //Instantiate an array of UDP objects
    if(rover_quantity>0)
        udparray_ = new async_comm::UDP*[rover_quantity];

    //Fill udp objects into the array.
    for(int i = 0; i < rover_quantity; i++) {
        std::cerr<<"Initializing Base to Rover "<<std::to_string(i+1)<<" UDP\n";

        //Create pointer to UDP object within an array
        udparray_[i] = new async_comm::UDP(local_host[i], local_port[i], remote_host[i], remote_port[i]);

        if (!udparray_[i]->init())
        {
            throw std::runtime_error("Failed to initialize Base to Rover "+ std::to_string(i+1) +" receive UDP\n");
        }

        rtcm_.registerCallback([ this , i](uint8_t* buf, size_t size)
        {
	    send5(buf, size);
	    
            // DBG("Sending RTCM Data\n");
	    size_t encrypted_size;
	    const uint8_t *encrypted_buf;
	    encrypt(buf, size, &encrypted_buf, &encrypted_size);
            //this->udparray_[i]->send_bytes(encrypted_buf, encrypted_size);
        });

        ubx_.registerCallback(CLASS_NAV, NAV_POSECEF, [this, i](uint8_t _class, uint8_t _type, const ublox::UBX_message_t& msg, uint8_t f9pID=0)
        {
            ubx_.create_message(buffer, CLASS_NAV, NAV_POSECEF, msg, sizeof(NAV_POSECEF_t));
	    send5(buffer,  8+sizeof(NAV_POSECEF_t));

	    size_t encrypted_size;
	    const uint8_t *encrypted_buf;
	    encrypt(buffer, 8+sizeof(NAV_POSECEF_t), &encrypted_buf, &encrypted_size);
            //this->udparray_[i]->send_bytes(encrypted_buf, encrypted_size);
        });

        ubx_.registerCallback(CLASS_NAV, NAV_PVT, [this, i](uint8_t _class, uint8_t _type, const ublox::UBX_message_t& msg, uint8_t f9pID=0)
        {
            ubx_.create_message(buffer, CLASS_NAV, NAV_PVT, msg, sizeof(NAV_PVT_t));
	    send5(buffer,  8+sizeof(NAV_PVT_t));

	    size_t encrypted_size;
	    const uint8_t *encrypted_buf;
	    encrypt(buffer, 8+sizeof(NAV_PVT_t), &encrypted_buf, &encrypted_size);
            //this->udparray_[i]->send_bytes(encrypted_buf, encrypted_size);
        });

        ubx_.registerCallback(CLASS_NAV, NAV_VELECEF, [this, i](uint8_t _class, uint8_t _type, const ublox::UBX_message_t& msg, uint8_t f9pID=0)
        {
            ubx_.create_message(buffer, CLASS_NAV, NAV_VELECEF, msg, sizeof(NAV_VELECEF_t));
	    send5(buffer,  8+sizeof(NAV_VELECEF_t));

	    size_t encrypted_size;
	    const uint8_t *encrypted_buf;
	    encrypt(buffer, 8+sizeof(NAV_VELECEF_t), &encrypted_buf, &encrypted_size);
            //this->udparray_[i]->send_bytes(encrypted_buf, encrypted_size);
        });

        std::cerr<<"Initialized Base to Rover "+ std::to_string(i+1) +" UDP\n";
    }

    if(base_type=="stationary")
    {
        config_rtcm_msgs(1, 1, surveyacc, surveytime, constellation);
    }
    else
    {
        config_rtcm_msgs(1, 0, 0, 0, constellation);
    }
    config_gnss(constellation);
    config_ubx_msgs(0);
    // config_base(base_type, gps, glonas, beidou, galileo, surveytime, surveyacc);
    config_f9p(dynamic_model);
}

void UBLOX::initBrover(std::string local_host[], uint16_t local_port[],
                std::string base_host[], uint16_t base_port[],
                std::string rover_host[], uint16_t rover_port[],
                std::string base_type, int rover_quantity, 
                GNSS_CONSTELLATION_t constellation, uint8_t dynamic_model) {
                
                config_gnss(constellation);
                config_ubx_msgs(1);
                config_rtcm_msgs(1, 0, 0, 0, constellation);

                // Declare type as Brover. This is used by rtcm_complete_cb()
                  type_ = BROVER;


                  assert(udp_ == nullptr);
                  // Connect the rtcm_cb callback to forward data to the UBX serial port
                  rtcm_.registerCallback([this](uint8_t* buf, size_t size)
                  {
                      this->rtcm_complete_cb(buf, size);
                  });

                  // hook up UDP to listen to the base
                  /// TODO: configure ports and IP from cli
                  udp_ = new async_comm::UDP(local_host[0], local_port[0], base_host[0], base_port[0]);
                  udp_->register_receive_callback([this](const uint8_t* buf, size_t size)
                  {
                      this->udp_read_cb(buf, size);
                  });

                  if (!udp_->init())
                      throw std::runtime_error("Failed to initialize Brover receive UDP");

                  //Instantiate an array of UDP objects for rovers
                  udparray_ = new async_comm::UDP*[rover_quantity];
                  std::cerr<<"Rover Quantity: "<<rover_quantity<<"\n";

                    //Fill rover udp objects into the array.
                    for(int i = 0; i < rover_quantity; i++) {
                        std::cerr<<"Initializing Brover at "<< local_host[i+1]<<", "<<local_port[i+1]
                        <<" to Rover "<<std::to_string(i+1)<<" at "<< rover_host[i]<<", "<<rover_port[i]<<" UDP\n";

                        //Create pointer to UDP object within an array
                        udparray_[i] = new async_comm::UDP(local_host[i+1], local_port[i+1], rover_host[i], rover_port[i]);

                        if (!udparray_[i]->init())
                        {
                            throw std::runtime_error("Failed to initialize Brover to Rover "+ std::to_string(i+1) +"  UDP\n");
                        }

                            // hook up UDP to send data to the rovers
                        rtcm_.registerCallback([ this , i](uint8_t* buf, size_t size)
                        {
                            this->udparray_[i]->send_bytes(buf, size);
                        });

                        ubx_.registerCallback(CLASS_NAV, NAV_POSECEF, [this, i](uint8_t _class, uint8_t _type, const ublox::UBX_message_t& msg, uint8_t f9pID=0)
                        {
                            ubx_.create_message(buffer, CLASS_NAV, NAV_POSECEF, msg, sizeof(NAV_POSECEF_t));
                            this->udparray_[i]->send_bytes(buffer, 8+sizeof(NAV_POSECEF_t));
                        });

                        ubx_.registerCallback(CLASS_NAV, NAV_PVT, [this, i](uint8_t _class, uint8_t _type, const ublox::UBX_message_t& msg, uint8_t f9pID=0)
                        {
                            ubx_.create_message(buffer, CLASS_NAV, NAV_PVT, msg, sizeof(NAV_PVT_t));
                            this->udparray_[i]->send_bytes(buffer, 8+sizeof(NAV_PVT_t));
                        });

                        ubx_.registerCallback(CLASS_NAV, NAV_VELECEF, [this, i](uint8_t _class, uint8_t _type, const ublox::UBX_message_t& msg, uint8_t f9pID=0)
                        {
                            ubx_.create_message(buffer, CLASS_NAV, NAV_VELECEF, msg, sizeof(NAV_VELECEF_t));
                            this->udparray_[i]->send_bytes(buffer, 8+sizeof(NAV_VELECEF_t));
                        });

                      std::cerr<<"Initialized Brover to Rover "+ std::to_string(i+1) +" UDP\n";
                  }

                    // configure the base
                //   config_base(base_type, gps, glonas, beidou, galileo, 0, 0);

                    // configure the f9p
                  config_f9p(dynamic_model);
                  std::cerr<<"Initialized Brover\n";
}

UBLOX::~UBLOX()
{
    if (udp_)
        delete udp_;

    if (log_file_.is_open())
        log_file_.close();
}

void UBLOX::udp_read_cb(const uint8_t* encryptedbuf, size_t size)
{
    assert(type_ == ROVER || type_ == BROVER);

    // decrypt message
    static uint8_t buf[13000];
    if (!_encryption_initialized) {
	_encryption_initialized = init_encryption();
    }
    int bytes = size;
    unsigned char * ciphertext = (unsigned char *)encryptedbuf;
    aes_decrypt(_de_evp_cipher_ctx, ciphertext, &bytes, (unsigned char*)buf);
    // end decryption
    

    if(buf[0]==START_BYTE_1 && buf[1]==START_BYTE_2)
    {
        // DBG("Received baseveldata and size: %i\n", size);
        for (int i = 0; i < size; i++)
        {
            // DBG("buf[%i]=%i\n",i, buf[i]);
            ubx_.read_cb(buf[i], 1);
        }
    }
    else if(buf[0]==rtcm::START_BYTE)
    {
        // DBG("Received rtcm data\n");
        for (int i = 0; i < size; i++)
        {
            rtcm_.read_cb(buf[i]);
        }
    }
    else
    {
        DBG("Got something bad: %i\n", buf[0]);
    }
    
}

void UBLOX::serial_read_cb(const uint8_t *buf, size_t size)
{

    for (int i = 0; i < size; i++)
    {
        /// TODO: don't give parsers data they don't need
        if (ubx_.parsing_message())
        {
            ubx_.read_cb(buf[i]);
        }
        else if (rtcm_.parsing_message() && type_ != NONE)
        {
            rtcm_.read_cb(buf[i]);
        }
        else
        {
            ubx_.read_cb(buf[i]);
            rtcm_.read_cb(buf[i]);
        }
    }

    if (log_file_.is_open())
    {
        log_file_.write((const char*)buf, size);
    }
}

void UBLOX::rtcm_complete_cb(const uint8_t *buf, size_t size)
{
    assert (type_ == ROVER || type_ == BASE || type_ == BROVER);
    if (type_ == ROVER)
        serial_.send_bytes(buf, size);
    else if (type_ == BASE)
    {
	send5(buf, size);

	size_t encrypted_size;
	const uint8_t *encrypted_buf;
	encrypt((uint8_t*)buf, size, &encrypted_buf, &encrypted_size);
        //udp_->send_bytes(encrypted_buf, encrypted_size);
    }
    else if (type_ == BROVER) {
        serial_.send_bytes(buf, size);
      }

}
  void UBLOX::vector_math(double ned_1[], double ned_2[], double answer[]) {

    // For loop to calculate difference in NED values.
    for(int i = 0; i < 3; i++) {
      // Assign for NED respectively.
      answer[i] = ned_2[i]-ned_1[i];
    } // End for loop

    // Find distance from point 1 to point 2
    answer[3] = sqrt(pow(answer[0],2)+pow(answer[1],2)+pow(answer[2],2));

    // Roll = nan since we cannot calculate roll so we leave answer[4] untouched

    // Calculate pitch
    answer[5] = atan((-answer[2])/sqrt(pow(answer[0],2)+pow(answer[1],2)));

    // Calculate heading relative to true north
    if(answer[0]>0 && answer[1]>0) {  // Quadrant 1
      answer[6] = atan(answer[1]/answer[0]);
    }
    else if(answer[0]<0 && answer[1]>0) { // Quadrant 2
      answer[6] = atan(-answer[0]/answer[1])+PI/2;
    }
    else if(answer[0]<0 && answer[1]<0) { // Quadrant 3
      answer[6] = atan(-answer[1]/-answer[0]) + PI;
    }
    else {  // Quadrant 4
      answer[6] = atan(answer[0]/-answer[1])+PI*1.5;
    }

  } // End function vector_math

    CFG_VALGET_TUPLE_t UBLOX::cfgValGet(uint32_t cfgDataKey, uint8_t layer, uint16_t position, std::string filepath)
    {
        std::ofstream ofs;
        if(filepath!="")
            ofs.open(filepath, std::ios_base::trunc);
        std::vector<CFG_VALGET_t::response_t> cfgvec;
        std::vector<CFG_VALGET_t::response_t> allvalues;
        CFG_VAL_DBG_t debugging;
        do
        {
            CFG_VALGET_TUPLE_t results = ubx_.get_configuration(0, layer, position, cfgDataKey);
            debugging = std::get<0>(results);
            cfgvec = std::get<1>(results);
            for(uint8_t i=0; i<cfgvec.size(); i++)
            {
                allvalues.push_back(cfgvec[i]);
                if(filepath!="")
                {
                    if(i!=0)
                    {
                        ofs<<"\n";
                    }
                    ofs<<"Key: "<<cfgvec[i].cfgDataKey.keyID<<"\t";
                    ofs<<"Name: "<<cfgvec[i].keyName<<"\t";
                    ofs<<"Value: "<<cfgvec[i].cfgData.data<<"\t";
                    ofs<<"Layer: "<<int(cfgvec[i].layer)<<"\t";
                }
            }
            position = position+64;
            if(cfgvec.size()>=64)
            {
                ofs<<"\n";
            }
        } while (cfgvec.size()>=64);

        if(filepath!="")
        {
            ofs.close();
        }

        return {debugging, allvalues};
    }

    CFG_VALGET_TUPLE_t UBLOX::cfgValGet(const CFG_VALGET_t::request_t &request)
    {
        return ubx_.get_configuration(request.version, request.layer, request.position, request.cfgDataKey.keyID);
    }

    CFG_VAL_DBG_t UBLOX::cfgValDel(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
    {

        return ubx_.del_configuration(version, layer, cfgDataKey);
    }

    CFG_VAL_DBG_t UBLOX::cfgValSet(uint8_t version, uint8_t layer, uint64_t cfgData, uint32_t cfgDataKey)
    {

        return ubx_.configure(version, layer, cfgData, cfgDataKey);
    }

    navBbrMask_t UBLOX::reset(uint16_t navBbrMask, uint8_t resetMode)
    {
        union {
            navBbrMask_t bitfield;
            uint16_t number;
        };

        number = navBbrMask;
        // DBG("Bitfield Eph: %i", bitfield.eph);

        ubx_.reset(bitfield, resetMode);

        return bitfield;
    }

    void UBLOX::config_ubx_msgs(int relpos)
    {
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_PVT);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_POSECEF);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_VELECEF);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, relpos, CFG_VALSET_t::MSGOUT_RELPOSNED);
    }

    void UBLOX::config_rtcm_msgs(int hasRover, int stationary, int surveyacc, int surveytime, GNSS_CONSTELLATION_t constellation)
    {
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::RTCM_1074USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::RTCM_1084USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::RTCM_1094USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::RTCM_1124USB);
        
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*hasRover, CFG_VALSET_t::RTCM_4072_0USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*hasRover, CFG_VALSET_t::RTCM_4072_1USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*hasRover*constellation.gps_enable, CFG_VALSET_t::RTCM_1077USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*hasRover*constellation.glonas_enable, CFG_VALSET_t::RTCM_1087USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*hasRover*constellation.galileo_enable, CFG_VALSET_t::RTCM_1097USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*hasRover*constellation.beidou_enable, CFG_VALSET_t::RTCM_1127USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*hasRover*constellation.glonas_enable, CFG_VALSET_t::RTCM_1230USB);

        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*stationary, CFG_VALSET_t::RTCM_1005USB);
        ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*stationary, CFG_VALSET_t::MSGOUT_SVIN);
	if (DO_SURVEY_IN == 1) {
	    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*stationary, CFG_VALSET_t::TMODE_MODE);
	    // Survey in accuracy limit
	    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, surveyacc*stationary, CFG_VALSET_t::TMODE_SVIN_ACC_LIMIT);
	    // Survey in time limit
	    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, surveytime*stationary, CFG_VALSET_t::TMODE_SVIN_MIN_DUR);
	}
    }

    MON_VER_t UBLOX::getVersion()
    {
        return ubx_.getVersion().mon_ver;
    }

}
