/*
*
*
*
* NETWORK TOPOLOGY
                MAX_Y             MAX_X
        ___________\____________  /
        |   *                  | /
        |  *        *     *    |/
        |      *         *     |
       /|           *        * |
      / |   *           *      |
     /  |______________________|
 MIN_X              \
                  MIN_Y
*
*
*
*/

// calculer l'angle:
//                      Angles txAngles (receiverMobility->GetPosition (), txMobility->GetPosition
//                      ());

#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/object.h"
#include "ns3/olsr-helper.h"
#include "ns3/olsr-repositories.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/stats-module.h"
#include "ns3/wifi-module.h"

// #include "ns3/collision-avoidance-helper.h"

#include "ns3/MaliciousDroneHelper.h"
#include "ns3/SatelliteHelper.h"
#include "ns3/normal-drone-helper.h"

// #include "ns3/rng-seed-manager.h"

#include <ns3/itu-r-1411-los-propagation-loss-model.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/lr-wpan-spectrum-value-helper.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/single-model-spectrum-channel.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneAdhoc");

enum MethodName
{
    NormalCase,
    AuthenticWithSpoofing,
    Spoofing10,
    Spoofing20,
    Spoofing30,
    OneSatellite,
    AllSatellite,
    Burglary,
    AbsolutePower,
    AbsolutePower_CN0
};

struct limite
{
    double MaxX;
    double MaxY;
    double MaxZ;

    double MinX;
    double MinY;
    double MinZ;

    int NbrOfUAV;

    double Speed;
};

std::string averages = "";

class DroneExperiment : public Object
{
    // Constructor and Deconstructor
  public:
    static TypeId GetTypeId(void);

    DroneExperiment();
    ~DroneExperiment();

    // Runner
  public:
    std::vector<Gnuplot2dDataset> Run(MethodName method, bool Dynamic, bool onStateChange);

    // Configuration Helpers
    void SetDefaults();
    void Configure();

    void startSatelliteApp(Ptr<Node> satellite,
                           Ptr<LrWpanNetDevice> dev_sattelite,
                           double TxPower,
                           uint32_t PRN);
    void CreateNodesLrWPAN(int NbrUAV);
    void AffectationDesAddress_LrWPAN(uint32_t debut, uint32_t fin);

    void AffectationDesAddress_GPS_PRN1(uint32_t debut, uint32_t fin, uint32_t PRN_index);
    void InstallConstantMobilityLrwpan(uint32_t debut, uint32_t fin, double x, double y, double z);
    void InstallRandomMobilityLrWPAN(uint32_t debut, uint32_t fin, limite N1, limite N2, limite N3);
    void InstallHierarchicalMobilityModelLrwpan(uint32_t debut,
                                                uint32_t fin,
                                                limite LimiteG1); // + gps

    void InstallHierarchicalMobilityLrwpanWithGridAllocotar(uint32_t debut,
                                                            uint32_t fin,
                                                            limite LimiteG1,
                                                            limite LimiteG2,
                                                            limite LimiteG3);

    void InitialisationChanelLrwpan();
    void InitialisationChanelGPS();
    void InstallChannelGPS(uint32_t debut, uint32_t fin);
    void InstallChannelLrwpan(uint32_t debut, uint32_t fin);

    void InstallDevicesLrwpan(uint32_t debut, uint32_t fin);
    void InstallDevicesGPS(uint32_t debut, uint32_t fin);

    void InstallApps(uint32_t debut, uint32_t fin, uint32_t type);

    void Create12Satellite(double TxPower);

    void CreateNSatellite(int NbrSatellite, double TxPower, double StartTime);

    void CreateOneSatellite(double x,
                            double y,
                            double z,
                            uint32_t PRN,
                            double TxPower,
                            double StartTime);
    void AddOneUAV(uint32_t DroneType,
                   int MobilityType,
                   double x,
                   double y,
                   double z,
                   double spped);

    void ConnectCallbacks();

    void Cn0VersusTSP(double, double);
    void Alert(double, double, double);

    void Detection(double, double, double);

    // Conversion Helpers

    Vector GetNext(uint32_t m_current,
                   int m_layoutType,
                   double m_xMin,
                   double m_yMin,
                   double m_deltaX,
                   double m_deltaY,
                   uint32_t NbrMaxParLigne);
    std::string UniformRange(uint32_t min, uint32_t max);
    std::string WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY);
    double Percentage(int amount, int outOf);

    // Callbacks
  private:
    void RecordStats();
    Ptr<Node> satellite;
    Ptr<LrWpanNetDevice> dev_sattelite;
    // Packet counters
    double m_True_Positive = 0;
    double m_False_Positive = 0;

    double m_False_Negative = 0;
    double m_True_Negative = 0;

    int m_packetsSent;
    double m_SpoofingSatellitePower;
    int cp_packetsSent;
    int cp_packetsReceived;

    double m_ReceivingSatellitepower;
    double m_CN0;

    // Power management shared memory
    std::vector<double> m_phyStats;
    std::vector<Ptr<WifiPhy>> m_phyList;

    // Power management statistics
    double m_maxStat;
    double m_minStat;

    // Power management configs
    bool m_onStateChange;
    bool m_Dynamic;

    // Power management configs

    MethodName m_method;
    std::string m_methodString;

    // Drone info
    NodeContainer m_drones;
    uint32_t m_numDrones = 0;
    ApplicationContainer m_DroneApps;
    ApplicationContainer m_SatelliteApps;
    ApplicationContainer m_SpoofingSatelliteApps;

    /********************************************************************************/
    // lrwpan
    std::vector<Ptr<LrWpanNetDevice>> m_NetDevicesTable;
    std::vector<Ptr<ConstantPositionMobilityModel>> m_ConstantPositionMobilityTable;
    std::vector<Ptr<RandomWalk2dMobilityModel>> m_RandomMobilityTable;

    EventId m_sendEvent;

    /******************************************************************************/

    Ptr<MultiModelSpectrumChannel> channel;
    Ptr<ItuR1411LosPropagationLossModel> model;
    Ptr<ConstantSpeedPropagationDelayModel> delayModel;

    /******************************************************************************/
    // gps receiver
    Ptr<MultiModelSpectrumChannel> GPSchannel;
    Ptr<FriisPropagationLossModel> GPSmodel;
    Ptr<ConstantSpeedPropagationDelayModel> GPSdelayModel;

    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN0;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN1;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN2;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN3;

    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN4;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN5;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN6;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN7;

    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN8;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN9;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN10;
    std::vector<Ptr<LrWpanNetDevice>> m_GPS_device_Table_PRN11;

    // satellite :
    uint32_t m_numSatellites = 0;
    NodeContainer m_satellites;
    // Ptr<LrWpanNetDevice> dev_sattelite ;
    std::vector<Ptr<LrWpanNetDevice>> m_NetDevicesSatelliteTable;
    /************************************************************************************/
    // Drone Groupe1 info (in meters)
    uint32_t m_maxX;
    uint32_t m_minX;
    uint32_t m_maxY;
    uint32_t m_minY;

    // Drone communication info
    uint32_t m_PacketSize;      // bytes
    double m_DataUavFrequency;  // seconds
    uint32_t m_imagePacketSize; //
    double m_imageFrequency;    // seconds

    // PHY info
    std::string m_phyMode;

    // Throughput
    Gnuplot2dDataset m_success_rate;
    Gnuplot2dDataset m_cn0VersusAbsolutPower;
    Gnuplot2dDataset m_authenticSignal;
    uint32_t m_dataReceived;
    uint32_t m_dataSent;
    uint32_t cp_dataReceived;
    uint32_t cp_dataSent;

    double txPowerLrwpan = 10;
    uint32_t channelNumber = 11;
    uint32_t GPSchannelNumber = 11;

    int UAV_CONSTANTE_MOBILITY = 0;
    int UAV_RANDOM_WALK_MOBILITY = 1;
    int UAV_HERARCHICAL_MOBILITY = 2;
    int UAV_GRID_HERARCHICAL_MOBILITY = 3;

    uint32_t UAV_GOOD = 1;
    uint32_t UAV_BAD = 2;
    uint32_t UAV_VICTIM = 3;
};

TypeId
DroneExperiment::GetTypeId(void)
{
    NS_LOG_INFO("bada debut  GetTypeId");
    static TypeId tid =
        TypeId("ns3::DroneExperiment")
            .SetParent<Object>()
            .SetGroupName("Experiments")
            .AddConstructor<DroneExperiment>()
            .AddAttribute("CollisionPacketSize",
                          "The size of the packets used for the CollisionAvoidanceApp.",
                          UintegerValue(1024),
                          MakeUintegerAccessor(&DroneExperiment::m_PacketSize),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute(
                "CollisionPacketFrequency",
                "The interval in seconds of how often CollisionAvoidance packets should be sent.",
                DoubleValue(1),
                MakeDoubleAccessor(&DroneExperiment::m_DataUavFrequency),
                MakeDoubleChecker<double>())
            .AddAttribute(
                "ImagePacketSize",
                "The size of the packets used for the ImageManagementApp (app not yet created).",
                UintegerValue(5 * 1024 * 1024),
                MakeUintegerAccessor(&DroneExperiment::m_imagePacketSize),
                MakeUintegerChecker<uint32_t>(1))
            .AddAttribute(
                "ImagePacketFrequency",
                "The interval in seconds of how often ImageManagementApp packets should be sent.",
                DoubleValue(10.0),
                MakeDoubleAccessor(&DroneExperiment::m_imageFrequency),
                MakeDoubleChecker<double>())
            .AddAttribute("NumDrones",
                          "The number of drones to use in the experiment.",
                          UintegerValue(30),
                          MakeUintegerAccessor(&DroneExperiment::m_numDrones),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("PHYmode",
                          "The PHY mode to use for the PHY layer.",
                          StringValue("OfdmRate6Mbps"),
                          MakeStringAccessor(&DroneExperiment::m_phyMode),
                          MakeStringChecker())
            .AddAttribute("MaxX",
                          "The right most wall of the drone environment.",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&DroneExperiment::m_maxX),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("MinX",
                          "The left most wall of the drone environment.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&DroneExperiment::m_minX),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("MaxY",
                          "The upper most wall of the drone environment.",
                          UintegerValue(1000),
                          MakeUintegerAccessor(&DroneExperiment::m_maxY),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("MinY",
                          "The bottom most wall of the drone environment.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&DroneExperiment::m_minY),
                          MakeUintegerChecker<uint32_t>(1));

    NS_LOG_INFO("bada fin  GetTypeId");

    return tid;
}

// 30 100 0 100 0 1024 0.5 5*1024*1024 10.0
DroneExperiment::DroneExperiment()
    : m_numDrones(10),
      m_maxX(1000),
      m_minX(0),
      m_maxY(1000),
      m_minY(0),
      m_PacketSize(4),
      m_DataUavFrequency(1),
      m_imagePacketSize(5 * 1024 * 1024),
      m_imageFrequency(10.0),
      m_phyMode("OfdmRate6Mbps")
{
    NS_LOG_INFO("bada debut +fin   DroneExperiment initialiser automatique");
}

DroneExperiment::~DroneExperiment()
{
    NS_LOG_INFO("bada debut +fin   DroneExperiment sans initialisation");
}

void
DroneExperiment::SetDefaults()
{
    NS_LOG_INFO("bada debut SetDefaults");

    // Make sure these are reset just in case the same Experiment is run twice
    m_packetsSent = 0;
    m_SpoofingSatellitePower = 50.1;
    m_dataReceived = 0;
    m_dataSent = 0;
    cp_packetsSent = 0;
    cp_packetsReceived = 0;
    cp_dataReceived = 0;
    cp_dataSent = 0;

    m_True_Negative = 0;
    m_True_Positive = 0;
    m_False_Negative = 0;
    m_False_Positive = 0;

    m_phyList = {};
    m_phyStats = {};

    // Create enough space for all drones
    m_phyList.resize(0);
    m_phyList.resize(m_numDrones);
    m_phyStats.resize(0);
    m_phyStats.resize(m_numDrones);

    switch (m_method)
    {
    case AuthenticWithSpoofing:
        m_methodString = "Authentic* + Spoofing (Na=7  Ns=10)";
        break;
    case Spoofing10:
        m_methodString = "Spoofing* + Authentic (Na=7   Ns=10)";
        break;
    case Spoofing20:
        m_methodString = "Spoofing* + Authentic (Na=7   Ns=20)";
        break;
    case Spoofing30:
        m_methodString = "Spoofing* + Authentic (Na=7   Ns=30)";
        break;
    case OneSatellite:
        m_methodString = "Max: authentic* signal (Na=1  Ns=0)";
        break;
    case AllSatellite:
        m_methodString = "Min: authentic* signal (Na=14  Ns=0) ";
        break;

    case NormalCase:
        m_methodString = "Authentic* signal avrg  (Na=7  Ns=0)";
        break;

    case Burglary:
        m_methodString = "Burglary";
        break;
    case AbsolutePower:
        m_methodString = "AbsolutePower";
        break;
    case AbsolutePower_CN0:
        m_methodString = "AbsolutePower_CN0";
        break;
    }

    m_drones = NodeContainer();

    m_cn0VersusAbsolutPower = Gnuplot2dDataset(m_methodString);
    m_cn0VersusAbsolutPower.SetStyle(Gnuplot2dDataset::POINTS);
    m_cn0VersusAbsolutPower.SetExtra("set key font \",20\" ");

    m_success_rate = Gnuplot2dDataset(m_methodString);

    // m_success_rate.SetStyle (Gnuplot2dDataset::POINTS);

    m_NetDevicesTable.clear();
    m_ConstantPositionMobilityTable.clear();
    m_RandomMobilityTable.clear();

    m_GPS_device_Table_PRN0.clear();
    m_GPS_device_Table_PRN1.clear();
    m_GPS_device_Table_PRN2.clear();
    m_GPS_device_Table_PRN3.clear();

    m_GPS_device_Table_PRN4.clear();
    m_GPS_device_Table_PRN5.clear();
    m_GPS_device_Table_PRN6.clear();
    m_GPS_device_Table_PRN7.clear();

    m_GPS_device_Table_PRN8.clear();
    m_GPS_device_Table_PRN9.clear();
    m_GPS_device_Table_PRN10.clear();
    m_GPS_device_Table_PRN11.clear();

    m_NetDevicesSatelliteTable.clear();
    m_satellites = NodeContainer();
    ;

    m_numSatellites = 0;
    m_numDrones = 0;
    NS_LOG_INFO("bada fin SetDefaults");
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::CreateNodesLrWPAN(int NbrUAV)
{
    NS_LOG_INFO("bada debut CreateNodes");

    // pour l'utilisation des fonctions random en ns3

    RngSeedManager::SetSeed(4); //
    RngSeedManager::SetRun(6);
    m_numDrones = NbrUAV;
    m_drones.Create(NbrUAV);

    NS_LOG_INFO("bada fin CreateNodes");
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::AffectationDesAddress_LrWPAN(uint32_t debut, uint32_t fin)
{
    // creation d'un model pour les adresses
    char adresse[5] = {'0', '0', ':', '0', '0'};

    // creation de  la carte reseau pour chaque drone (adresse MAC sur 16 bits / 2 octets de 00:00
    // jusqua FF:FF)
    //  + stockage des adresses MAC16 dans un vecteur de pointeur   m_NetDevicesTable de
    //  LrWpanNetDevice
    Ptr<LrWpanNetDevice> dev_i;
    for (uint32_t i = debut; i < fin; i++)
    {
        std::stringstream str;
        str << (i);
        dev_i = CreateObject<LrWpanNetDevice>();

        if (i < 10)
        {
            adresse[4] = str.str()[0];
            dev_i->SetAddress(
                Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
            m_NetDevicesTable.push_back(dev_i);
        }
        else
        {
            adresse[4] = str.str()[1];
            adresse[3] = str.str()[0];

            dev_i->SetAddress(Mac16Address(adresse));
            m_NetDevicesTable.push_back(dev_i);
        }
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::AffectationDesAddress_GPS_PRN1(uint32_t debut, uint32_t fin, uint32_t PRN_index)
{
    uint32_t authenticPRN[] = {
        4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
        2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
        3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
        549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
        28594359,   349684609,  9999122,    224443546};

    // creation d'un model pour les adresses GPS
    char adresse[5] = {'7', '7', ':', '7', '7'};

    /********************************************* creation du device pour PRN
     * 0******************************************/

    // creation de  la carte reseau pour chaque drone (adresse MAC sur 16 bits / 2 octets de 00:00
    // jusqua FF:FF)
    //  + stockage des adresses MAC16 dans un vecteur de pointeur   m_NetDevicesTable de
    //  LrWpanNetDevice
    Ptr<LrWpanNetDevice> GPSdev_PRN0;
    Ptr<LrWpanNetDevice> GPSdev_PRN1;
    Ptr<LrWpanNetDevice> GPSdev_PRN2;
    Ptr<LrWpanNetDevice> GPSdev_PRN3;
    Ptr<LrWpanNetDevice> GPSdev_PRN4;
    Ptr<LrWpanNetDevice> GPSdev_PRN5;
    Ptr<LrWpanNetDevice> GPSdev_PRN6;
    Ptr<LrWpanNetDevice> GPSdev_PRN7;
    Ptr<LrWpanNetDevice> GPSdev_PRN8;
    Ptr<LrWpanNetDevice> GPSdev_PRN9;
    Ptr<LrWpanNetDevice> GPSdev_PRN10;
    Ptr<LrWpanNetDevice> GPSdev_PRN11;

    for (uint32_t i = debut; i < fin; i++)
    {
        std::stringstream str;
        str << (i);

        GPSdev_PRN0 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN1 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN2 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN3 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN4 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN5 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN6 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN7 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN8 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN9 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN10 = CreateObject<LrWpanNetDevice>();
        GPSdev_PRN11 = CreateObject<LrWpanNetDevice>();

        GPSdev_PRN0->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN1->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN2->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN3->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN4->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN5->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN6->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN7->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN8->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN9->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN10->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        GPSdev_PRN11->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i

        GPSdev_PRN0->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN1->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN2->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN3->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN4->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN5->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN6->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN7->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN8->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN9->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN10->GetPhy()->m_isSatellite = 1;
        GPSdev_PRN11->GetPhy()->m_isSatellite = 1;

        GPSdev_PRN0->GetPhy()->m_PrnChannel = authenticPRN[0];
        GPSdev_PRN1->GetPhy()->m_PrnChannel = authenticPRN[1];
        GPSdev_PRN2->GetPhy()->m_PrnChannel = authenticPRN[2];
        GPSdev_PRN3->GetPhy()->m_PrnChannel = authenticPRN[3];
        GPSdev_PRN4->GetPhy()->m_PrnChannel = authenticPRN[4];
        GPSdev_PRN5->GetPhy()->m_PrnChannel = authenticPRN[5];
        GPSdev_PRN6->GetPhy()->m_PrnChannel = authenticPRN[6];
        GPSdev_PRN7->GetPhy()->m_PrnChannel = authenticPRN[7];
        GPSdev_PRN8->GetPhy()->m_PrnChannel = authenticPRN[8];
        GPSdev_PRN9->GetPhy()->m_PrnChannel = authenticPRN[9];
        GPSdev_PRN10->GetPhy()->m_PrnChannel = authenticPRN[10];
        GPSdev_PRN11->GetPhy()->m_PrnChannel = authenticPRN[11];

        // GPSdev_PRN0->GetPhy()->SetSatelliteReceptionMode(FIFO);
        // GPSdev_PRN0->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN0->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN1->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN2->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN3->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN4->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN5->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN6->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN7->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN8->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN9->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN10->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);
        GPSdev_PRN11->GetPhy()->SetSatelliteReceptionMode(MaxPeakRAZ);

        m_GPS_device_Table_PRN0.push_back(GPSdev_PRN0);
        m_GPS_device_Table_PRN1.push_back(GPSdev_PRN1);
        m_GPS_device_Table_PRN2.push_back(GPSdev_PRN2);
        m_GPS_device_Table_PRN3.push_back(GPSdev_PRN3);
        m_GPS_device_Table_PRN4.push_back(GPSdev_PRN4);
        m_GPS_device_Table_PRN5.push_back(GPSdev_PRN5);
        m_GPS_device_Table_PRN6.push_back(GPSdev_PRN6);
        m_GPS_device_Table_PRN7.push_back(GPSdev_PRN7);
        m_GPS_device_Table_PRN8.push_back(GPSdev_PRN8);
        m_GPS_device_Table_PRN9.push_back(GPSdev_PRN9);
        m_GPS_device_Table_PRN10.push_back(GPSdev_PRN10);
        m_GPS_device_Table_PRN11.push_back(GPSdev_PRN11);
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallConstantMobilityLrwpan(uint32_t debut,
                                               uint32_t fin,
                                               double x,
                                               double y,
                                               double z)
{
    // affecter la mobility de chaque UAV (cote Lrwpan)
    Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();
    for (uint32_t i = debut; i < fin; i++)
    {
        m_NetDevicesTable[i]->GetPhy()->SetMobility(mob0);
        mob0->SetPosition(Vector(x, y, z));
    }

    // recuperer la mobility qu'on a deja affecter afin de la visualiser sur netanim
    Ptr<ConstantPositionMobilityModel>
        mm0; // sert a recuperer la mobility de l'uavi pour le deploimenet en netanim
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    for (uint32_t i = debut; i < fin; i++)
    {
        mobility.Install(m_drones.Get(i));
        mm0 = m_drones.Get(i)->GetObject<ConstantPositionMobilityModel>();
        mm0->SetPosition(Vector(x, y, z));
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallRandomMobilityLrWPAN(uint32_t debut,
                                             uint32_t fin,
                                             limite LimiteG1,
                                             limite LimiteG2,
                                             limite LimiteG3)
{
    std::cout << "	**************************** debut  "
                 "InstallRandomMobilityLrWPAN******************************* \n";

    /******************************** netanim
     * mobility***********************************************/

    // Groupe 1 : mobility pour NetAnim
    MobilityHelper mobility1;
    mobility1.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                   "X",
                                   StringValue(UniformRange(LimiteG1.MinX, LimiteG1.MaxX)),
                                   "Y",
                                   StringValue(UniformRange(LimiteG1.MinY, LimiteG1.MaxY)),
                                   "Z",
    // Tahar :: Changed
                                   StringValue(UniformRange(LimiteG1.MinZ, LimiteG1.MaxZ)));
    mobility1.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel",
        "Mode",
        StringValue("Time"),
        "Time",
        StringValue("0.5s"),
        "Speed",
        StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG1.Speed) + "]"),
        "Bounds",
        StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));

    for (uint32_t i = debut; i < (debut + LimiteG1.NbrOfUAV); i++)
    {
        mobility1.Install(m_drones.Get(i));
    }

    // Groupe 2 : mobility pour NetAnim
    MobilityHelper mobility2;
    mobility2.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                   "X",
                                   StringValue(UniformRange(LimiteG2.MinX, LimiteG2.MaxX)),
                                   "Y",
                                   StringValue(UniformRange(LimiteG2.MinY, LimiteG2.MaxY)),
                                   "Z",
                                   StringValue(UniformRange(LimiteG2.MinZ, LimiteG2.MaxZ)));
    mobility2.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel",
        "Mode",
        StringValue("Time"),
        "Time",
        StringValue("0.5s"),
        "Speed",
        StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG2.Speed) + "]"),
        "Bounds",
        StringValue(WalkBounds(LimiteG2.MinX, LimiteG2.MaxX, LimiteG2.MinY, LimiteG2.MaxY)));
    for (uint32_t i = (debut + LimiteG1.NbrOfUAV);
         i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV);
         i++)
    {
        mobility2.Install(m_drones.Get(i));
    }

    // Groupe 3 : mobility pour NetAnim
    MobilityHelper mobility3;
    mobility3.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                   "X",
                                   StringValue(UniformRange(LimiteG3.MinX, LimiteG3.MaxX)),
                                   "Y",
                                   StringValue(UniformRange(LimiteG3.MinY, LimiteG3.MaxY)),
                                   "Z",
                                   StringValue(UniformRange(LimiteG3.MinZ, LimiteG3.MaxZ)));
    mobility3.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel",
        "Mode",
        StringValue("Time"),
        "Time",
        StringValue("0.5s"),
        "Speed",
        StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(LimiteG3.Speed) + "]"),
        "Bounds",
        StringValue(WalkBounds(LimiteG3.MinX, LimiteG3.MaxX, LimiteG3.MinY, LimiteG3.MaxY)));
    for (uint32_t i = (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV);
         i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV + LimiteG3.NbrOfUAV);
         i++)
    {
        mobility3.Install(m_drones.Get(i));
    }

    std::cout << "	**************************** mileu (reste device)  "
                 "InstallRandomMobilityLrWPAN******************************* \n";

    /*********************************lrwpan mobility**********************************************/

    // Groupe1: install the mobility in the devices [drone 0 ..N1]

    Ptr<RandomWalk2dMobilityModel> mob1 = CreateObject<RandomWalk2dMobilityModel>();
    mob1->SetAttribute("Mode", StringValue("Time"));
    mob1->SetAttribute("Time", StringValue("0.5s"));
    mob1->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"));
    mob1->SetAttribute(
        "Bounds",
        StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));
    std::cout << "	avant la boucle \n";

    for (uint32_t i = debut; i < (debut + LimiteG1.NbrOfUAV); i++)
    {
        std::cout << "	dans la boucle \n";

        m_NetDevicesTable[i]->GetPhy()->SetMobility(mob1);
        m_RandomMobilityTable.push_back(mob1);
    }
    std::cout << "	apres la boucle \n";

    // Groupe2: install the mobility in the devices [drone N1+ 1 ..N2]

    Ptr<RandomWalk2dMobilityModel> mob2 = CreateObject<RandomWalk2dMobilityModel>();
    mob2->SetAttribute("Mode", StringValue("Time"));
    mob2->SetAttribute("Time", StringValue("0.5s"));
    mob2->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"));
    mob1->SetAttribute(
        "Bounds",
        StringValue(WalkBounds(LimiteG2.MinX, LimiteG2.MaxX, LimiteG2.MinY, LimiteG2.MaxY)));

    for (uint32_t i = (debut + LimiteG1.NbrOfUAV);
         i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV);
         i++)
    {
        m_NetDevicesTable[i]->GetPhy()->SetMobility(mob2);
        m_RandomMobilityTable.push_back(mob1);
    }

    // Groupe3: install the mobility in the devices [drone N2+ 1 ..m_numDrones (N3)]

    Ptr<RandomWalk2dMobilityModel> mob3 = CreateObject<RandomWalk2dMobilityModel>();
    mob3->SetAttribute("Mode", StringValue("Time"));
    mob3->SetAttribute("Time", StringValue("0.5s"));
    mob3->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"));
    mob1->SetAttribute(
        "Bounds",
        StringValue(WalkBounds(LimiteG3.MinX, LimiteG3.MaxX, LimiteG3.MinY, LimiteG3.MaxY)));

    for (uint32_t i = (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV);
         i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV + LimiteG3.NbrOfUAV);
         i++)
    {
        m_NetDevicesTable[i]->GetPhy()->SetMobility(mob3);
        m_RandomMobilityTable.push_back(mob1);
    }
    std::cout << "	**************************** fin  "
                 "InstallRandomMobilityLrWPAN******************************* \n";
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallHierarchicalMobilityModelLrwpan(uint32_t debut,
                                                        uint32_t fin,
                                                        limite LimiteG1)
{
    for (uint32_t i = debut; i < fin; i++)
    {
        MobilityHelper mobility;

        // le 1er type de mobility: constante position
        Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();

        // le 2eme type de mobility: Random walk 2d
        Ptr<RandomWalk2dMobilityModel> mob1 = CreateObject<RandomWalk2dMobilityModel>();
        mob1->SetAttribute("Mode", StringValue("Time"));
        mob1->SetAttribute("Time", StringValue("0.5s"));
        mob1->SetAttribute("Speed",
                           StringValue("ns3::ConstantRandomVariable[Constant=" +
                                       std::to_string(LimiteG1.Speed) + "]"));
        mob1->SetAttribute(
            "Bounds",
            StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));
        m_RandomMobilityTable.push_back(mob1);
        // affecter les 2 mobility
        mobility.SetMobilityModel("ns3::HierarchicalMobilityModel",
                                  "Child",
                                  PointerValue(mob1),
                                  "Parent",
                                  PointerValue());

        mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                      "X",
                                      StringValue(UniformRange(LimiteG1.MinX, LimiteG1.MaxX)),
                                      "Y",
                                      StringValue(UniformRange(LimiteG1.MinY, LimiteG1.MaxY)),
                                      "Z",
                                      StringValue(UniformRange(LimiteG1.MinZ, LimiteG1.MaxZ)));

        // install mobility pour netanim
        mobility.Install(m_drones.Get(i));

        // install mobility pour les devices lrwpan
        m_NetDevicesTable[i]->GetPhy()->SetMobility(mob1);

        // install gps receiver mobility, il prend les meme position que l'uav
        //  m_GPS_device_Table_PRN0[i]->GetPhy ()->SetMobility (mob1);

        // mob1->SetAttribute("Speed", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallHierarchicalMobilityLrwpanWithGridAllocotar(uint32_t debut,
                                                                    uint32_t fin,
                                                                    limite LimiteG1,
                                                                    limite LimiteG2,
                                                                    limite LimiteG3)
{
    // groupe1 : installer les 2 types de mobility
    for (uint32_t i = debut; i < (debut + LimiteG1.NbrOfUAV); i++)
    {
        MobilityHelper mobility;

        // le 1er type de mobility: constante position
        Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();

        // le 2eme type de mobility: Random walk 2d
        Ptr<RandomWalk2dMobilityModel> mob1 = CreateObject<RandomWalk2dMobilityModel>();
        mob1->SetAttribute("Mode", StringValue("Time"));
        mob1->SetAttribute("Time", StringValue("0.5s"));
        mob1->SetAttribute("Speed",
                           StringValue("ns3::ConstantRandomVariable[Constant=" +
                                       std::to_string(LimiteG1.Speed) + "]"));
        mob1->SetAttribute(
            "Bounds",
            StringValue(WalkBounds(LimiteG1.MinX, LimiteG1.MaxX, LimiteG1.MinY, LimiteG1.MaxY)));
        m_RandomMobilityTable.push_back(mob1);
        // affecter les 2 mobility
        mobility.SetMobilityModel("ns3::HierarchicalMobilityModel",
                                  "Child",
                                  PointerValue(mob1),
                                  "Parent",
                                  PointerValue());

        mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                      "X",
                                      StringValue(UniformRange(LimiteG1.MinX, LimiteG1.MaxX)),
                                      "Y",
                                      StringValue(UniformRange(LimiteG1.MinY, LimiteG1.MaxY)),
                                      "Z",
                                      StringValue(UniformRange(LimiteG1.MinZ, LimiteG1.MaxZ)));

        // install mobility pour netanim
        mobility.Install(m_drones.Get(i));
    }

    std::cout << "	****************************set grid position alocator  "
                 "g1******************************* \n";

    /****************************set grid position alocator
     * **********************************************/

    int TypeDeDistribution = 1; // 1 : par ligne , 2 : par column
    double xStart = LimiteG1.MinX;
    double yStart = LimiteG1.MinY;
    double distanceX = 1000;
    double distanceY = 2000;
    uint32_t nbrMaxParLigne = 3;

    for (uint32_t i = debut; i < (debut + LimiteG1.NbrOfUAV); i++)
    {
        Ptr<HierarchicalMobilityModel> Mob =
            m_drones.Get(i)->GetObject<HierarchicalMobilityModel>();

        Mob->SetPosition(
            GetNext(i, TypeDeDistribution, xStart, yStart, distanceX, distanceY, nbrMaxParLigne));

        // install mobility pour les devices lrwpan
        m_NetDevicesTable[i]->GetPhy()->SetMobility(m_RandomMobilityTable[i]);
    }

    // groupe2 : installer les 2 types de mobility
    for (uint32_t i = (debut + LimiteG1.NbrOfUAV);
         i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV);
         i++)
    {
        MobilityHelper mobility;

        // le 1er type de mobility: constante position
        Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();

        // le 2eme type de mobility: Random walk 2d
        Ptr<RandomWalk2dMobilityModel> mob1 = CreateObject<RandomWalk2dMobilityModel>();
        mob1->SetAttribute("Mode", StringValue("Time"));
        mob1->SetAttribute("Time", StringValue("0.5s"));
        mob1->SetAttribute("Speed",
                           StringValue("ns3::ConstantRandomVariable[Constant=" +
                                       std::to_string(LimiteG2.Speed) + "]"));
        mob1->SetAttribute(
            "Bounds",
            StringValue(WalkBounds(LimiteG2.MinX, LimiteG2.MaxX, LimiteG2.MinY, LimiteG2.MaxY)));
        m_RandomMobilityTable.push_back(mob1);
        // affecter les 2 mobility
        mobility.SetMobilityModel("ns3::HierarchicalMobilityModel",
                                  "Child",
                                  PointerValue(mob1),
                                  "Parent",
                                  PointerValue());

        mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                      "X",
                                      StringValue(UniformRange(LimiteG2.MinX, LimiteG2.MaxX)),
                                      "Y",
                                      StringValue(UniformRange(LimiteG2.MinY, LimiteG2.MaxY)),
                                      "Z",
                                      StringValue(UniformRange(LimiteG2.MinZ, LimiteG2.MaxZ)));

        // install mobility pour netanim
        mobility.Install(m_drones.Get(i));
    }

    std::cout << "	****************************set grid position alocator  "
                 "g2******************************* \n";

    /****************************set grid position alocator
     * **********************************************/

    int TypeDeDistribution2 = 1; // 1 : par ligne , 2 : par column
    double xStart2 = LimiteG2.MinX;
    double yStart2 = LimiteG2.MinY;
    double distanceX2 = 1000;
    double distanceY2 = 2000;
    uint32_t nbrMaxParLigne2 = 3;

    for (uint32_t i = (debut + LimiteG1.NbrOfUAV);
         i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV);
         i++)
    {
        Ptr<HierarchicalMobilityModel> Mob =
            m_drones.Get(i)->GetObject<HierarchicalMobilityModel>();

        Mob->SetPosition(GetNext(i,
                                 TypeDeDistribution2,
                                 xStart2,
                                 yStart2,
                                 distanceX2,
                                 distanceY2,
                                 nbrMaxParLigne2));

        // install mobility pour les devices lrwpan
        m_NetDevicesTable[i]->GetPhy()->SetMobility(m_RandomMobilityTable[i]);
    }

    // groupe3 : installer les 2 types de mobility
    for (uint32_t i = (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV);
         i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV + LimiteG3.NbrOfUAV);
         i++)
    {
        MobilityHelper mobility;
        std::cout << "	**************************** noeud " << i
                  << "******************************* \n";

        // le 1er type de mobility: constante position
        Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();

        // le 2eme type de mobility: Random walk 2d
        Ptr<RandomWalk2dMobilityModel> mob1 = CreateObject<RandomWalk2dMobilityModel>();
        mob1->SetAttribute("Mode", StringValue("Time"));
        mob1->SetAttribute("Time", StringValue("0.5s"));
        mob1->SetAttribute("Speed",
                           StringValue("ns3::ConstantRandomVariable[Constant=" +
                                       std::to_string(LimiteG3.Speed) + "]"));
        mob1->SetAttribute(
            "Bounds",
            StringValue(WalkBounds(LimiteG3.MinX, LimiteG3.MaxX, LimiteG3.MinY, LimiteG3.MaxY)));
        m_RandomMobilityTable.push_back(mob1);

        // affecter les 2 mobility
        mobility.SetMobilityModel("ns3::HierarchicalMobilityModel",
                                  "Child",
                                  PointerValue(mob1),
                                  "Parent",
                                  PointerValue());

        mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                      "X",
                                      StringValue(UniformRange(LimiteG3.MinX, LimiteG3.MaxX)),
                                      "Y",
                                      StringValue(UniformRange(LimiteG3.MinY, LimiteG3.MaxY)),
                                      "Z",
                                      StringValue(UniformRange(LimiteG3.MinZ, LimiteG3.MaxZ)));

        // install mobility pour netanim
        mobility.Install(m_drones.Get(i));
    }

    std::cout << "	****************************set grid position alocator  "
                 "g3******************************* \n";

    /****************************set grid position alocator
     ***********************************************/

    int TypeDeDistribution3 = 1; // 1 : par ligne , 2 : par column
    double xStart3 = LimiteG3.MinX;
    double yStart3 = LimiteG3.MinY;
    double distanceX3 = 500;
    double distanceY3 = 500;
    uint32_t nbrMaxParLigne3 = 3;

    for (uint32_t i = (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV);
         i < (debut + LimiteG1.NbrOfUAV + LimiteG2.NbrOfUAV + LimiteG3.NbrOfUAV);
         i++)
    {
        Ptr<HierarchicalMobilityModel> Mob =
            m_drones.Get(i)->GetObject<HierarchicalMobilityModel>();

        Mob->SetPosition(GetNext(i,
                                 TypeDeDistribution3,
                                 xStart3,
                                 yStart3,
                                 distanceX3,
                                 distanceY3,
                                 nbrMaxParLigne3));

        // install mobility pour les devices lrwpan
        m_NetDevicesTable[i]->GetPhy()->SetMobility(m_RandomMobilityTable[i]);
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InitialisationChanelLrwpan()

{
    channel = CreateObject<MultiModelSpectrumChannel>();
    model = CreateObject<ItuR1411LosPropagationLossModel>();
    delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(model);

    // si j'utilise delay model les resultats deviennent imprevisible
    // channel->SetPropagationDelayModel (delayModel);
}

void
DroneExperiment::InitialisationChanelGPS()

{
    GPSchannel = CreateObject<MultiModelSpectrumChannel>();

    GPSmodel = CreateObject<FriisPropagationLossModel>();
    GPSmodel->SetFrequency(1575420000);

    GPSdelayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
    GPSdelayModel->SetSpeed(299792458.0);

    GPSchannel->AddPropagationLossModel(GPSmodel);
    GPSchannel->SetPropagationDelayModel(GPSdelayModel);
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallChannelLrwpan(uint32_t debut, uint32_t fin)

{
    LrWpanSpectrumValueHelper svh;

    Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(2.0, channelNumber);

    // affectation de la puisance + channnel
    for (uint32_t i = debut; i < fin; i++)
    {
        m_NetDevicesTable[i]->SetChannel(channel);
        m_NetDevicesTable[i]->GetPhy()->SetTxPowerSpectralDensity(psd);
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallChannelGPS(uint32_t debut, uint32_t fin)

{
    LrWpanSpectrumValueHelper svh;

    Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(57.0, GPSchannelNumber); // 57.0 dbmw

    // affectation de la puisance + channnel
    for (uint32_t i = debut; i < fin; i++)
    {
        m_GPS_device_Table_PRN0[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN0[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN1[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN1[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN2[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN2[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN3[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN3[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN4[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN4[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN5[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN5[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN6[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN6[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN7[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN7[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN8[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN8[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN9[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN9[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN10[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN10[i]->GetPhy()->SetTxPowerSpectralDensity(psd);

        m_GPS_device_Table_PRN11[i]->SetChannel(GPSchannel);
        m_GPS_device_Table_PRN11[i]->GetPhy()->SetTxPowerSpectralDensity(psd);
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallDevicesLrwpan(uint32_t debut, uint32_t fin)
{
    // To complete configuration, a LrWpanNetDevice must be added to a node

    for (uint32_t i = debut; i < fin; i++)
    {
        m_drones.Get(i)->AddLrWpanNetDevice(m_NetDevicesTable[i]);
        m_drones.Get(i)->AddDevice(m_NetDevicesTable[i]);

        for (uint32_t j = debut; j < fin; j++)
        {
            if (j != i)
            {
                m_drones.Get(i)->AddWpanNetDeviceNeighbors(m_NetDevicesTable[j]);
            }
        }
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallDevicesGPS(uint32_t debut, uint32_t fin)
{
    // To complete configuration, a LrWpanNetDevice must be added to a node

    std::cout << "# Tahar : Start : InstallDevicesGPS \n ";

    for (uint32_t i = debut; i < fin; i++)
    {
        std::cout << " ## Tahar : Start : loop : InstallDevicesGPS \n ";

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN0[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN0[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN1[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN1[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN2[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN2[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN3[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN3[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN4[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN4[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN5[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN5[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN6[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN6[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN7[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN7[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN8[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN8[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN9[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN9[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN10[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN10[i]);

        m_drones.Get(i)->AddLrWpanNetDevice(m_GPS_device_Table_PRN11[i]);
        m_drones.Get(i)->AddDevice(m_GPS_device_Table_PRN11[i]);

        // Ptr<LrWpanNetDevice> MoiDevReceiver; // = CreateObject<LrWpanNetDevice>();
        Ptr<LrWpanNetDevice> MoiDevReceiver = Ptr<LrWpanNetDevice>(m_NetDevicesTable[i]);

        std::cout << " ## MoiDevReceiver " << MoiDevReceiver << "\n";
        std::cout << " ## MoiDevReceiver->GetMac()->GetShortAddress() "
                  << MoiDevReceiver->GetMac()->GetShortAddress() << "\n";
        // MoiDevReceiver = m_NetDevicesTable[i];

        Mac16Address Address = m_NetDevicesTable[i]->GetMac()->GetShortAddress();
        uint8_t* MonAdd = new uint8_t[2];
        Address.CopyTo(MonAdd);

        std::cout << " ## Tahar : Address.CopyTo(MonAdd) : InstallDevicesGPS \n ";

        m_GPS_device_Table_PRN0[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN1[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN2[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN3[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN4[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN5[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN6[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN7[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN8[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN9[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN10[i]->GetPhy()->m_attachedAdresse = MonAdd;
        m_GPS_device_Table_PRN11[i]->GetPhy()->m_attachedAdresse = MonAdd;
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::InstallApps(uint32_t debut, uint32_t fin, uint32_t type)
{
    NS_LOG_INFO("bada debut InstallApps");
    std::cout << " m_DataUavFrequency= : " << m_DataUavFrequency << "\n";
    uint32_t authenticPRN[] = {
        4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
        2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
        3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
        549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
        28594359,   349684609,  9999122,    224443546};

    if (type == UAV_VICTIM)
    {
        for (uint32_t i = debut; i < fin; i++)
        {
            NormalDroneHelper normalHelper;

            normalHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
            normalHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));
            normalHelper.SetAttribute("Victim", UintegerValue(1));

            normalHelper.SetAttribute("Mob", PointerValue(m_RandomMobilityTable[i]));
            normalHelper.SetAttribute("Phy", PointerValue(m_NetDevicesTable[i]->GetPhy()));
            normalHelper.SetAttribute("Mac", PointerValue(m_NetDevicesTable[i]->GetMac()));

            normalHelper.SetAttribute("GPS_PRN0", PointerValue(m_GPS_device_Table_PRN0[i]));
            normalHelper.SetAttribute("GPS_PRN1", PointerValue(m_GPS_device_Table_PRN1[i]));
            normalHelper.SetAttribute("GPS_PRN2", PointerValue(m_GPS_device_Table_PRN2[i]));
            normalHelper.SetAttribute("GPS_PRN3", PointerValue(m_GPS_device_Table_PRN3[i]));

            normalHelper.SetAttribute("GPS_PRN4", PointerValue(m_GPS_device_Table_PRN4[i]));
            normalHelper.SetAttribute("GPS_PRN5", PointerValue(m_GPS_device_Table_PRN5[i]));
            normalHelper.SetAttribute("GPS_PRN6", PointerValue(m_GPS_device_Table_PRN6[i]));
            normalHelper.SetAttribute("GPS_PRN7", PointerValue(m_GPS_device_Table_PRN7[i]));

            normalHelper.SetAttribute("GPS_PRN8", PointerValue(m_GPS_device_Table_PRN8[i]));
            normalHelper.SetAttribute("GPS_PRN9", PointerValue(m_GPS_device_Table_PRN9[i]));
            normalHelper.SetAttribute("GPS_PRN10", PointerValue(m_GPS_device_Table_PRN10[i]));
            normalHelper.SetAttribute("GPS_PRN11", PointerValue(m_GPS_device_Table_PRN11[i]));

            m_DroneApps = normalHelper.Install(m_drones.Get(i));
        }
    }

    if (type == UAV_GOOD)
    {
        for (uint32_t i = debut; i < fin; i++)
        {
            NormalDroneHelper normalHelper;

            normalHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
            normalHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));
            normalHelper.SetAttribute("Victim", UintegerValue(2));

            normalHelper.SetAttribute("Mob", PointerValue(m_RandomMobilityTable[i]));
            normalHelper.SetAttribute("Phy", PointerValue(m_NetDevicesTable[i]->GetPhy()));
            normalHelper.SetAttribute("Mac", PointerValue(m_NetDevicesTable[i]->GetMac()));

            normalHelper.SetAttribute("GPS_PRN0", PointerValue(m_GPS_device_Table_PRN0[i]));
            normalHelper.SetAttribute("GPS_PRN1", PointerValue(m_GPS_device_Table_PRN1[i]));
            normalHelper.SetAttribute("GPS_PRN2", PointerValue(m_GPS_device_Table_PRN2[i]));
            normalHelper.SetAttribute("GPS_PRN3", PointerValue(m_GPS_device_Table_PRN3[i]));

            normalHelper.SetAttribute("GPS_PRN4", PointerValue(m_GPS_device_Table_PRN4[i]));
            normalHelper.SetAttribute("GPS_PRN5", PointerValue(m_GPS_device_Table_PRN5[i]));
            normalHelper.SetAttribute("GPS_PRN6", PointerValue(m_GPS_device_Table_PRN6[i]));
            normalHelper.SetAttribute("GPS_PRN7", PointerValue(m_GPS_device_Table_PRN7[i]));

            normalHelper.SetAttribute("GPS_PRN8", PointerValue(m_GPS_device_Table_PRN8[i]));
            normalHelper.SetAttribute("GPS_PRN9", PointerValue(m_GPS_device_Table_PRN9[i]));
            normalHelper.SetAttribute("GPS_PRN10", PointerValue(m_GPS_device_Table_PRN10[i]));
            normalHelper.SetAttribute("GPS_PRN11", PointerValue(m_GPS_device_Table_PRN11[i]));

            if ((m_method == AbsolutePower))
            {
                normalHelper.SetAttribute("Methode", UintegerValue(1));
            }
            if ((m_method == AbsolutePower_CN0))
            {
                normalHelper.SetAttribute("Methode", UintegerValue(2));
            }

            if ((m_method == Burglary))
            {
                normalHelper.SetAttribute("Methode", UintegerValue(3));
            }

            m_DroneApps = normalHelper.Install(m_drones.Get(i));
        }
    }
    if (type == UAV_BAD)
    {
        for (uint32_t i = debut; i < fin; i++)
        {
            // affecter la'pplication malveillante au drone n0
            MaliciousDroneHelper malicioushelper;

            malicioushelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
            malicioushelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));

            malicioushelper.SetAttribute("Mob", PointerValue(m_RandomMobilityTable[i]));
            malicioushelper.SetAttribute("Phy", PointerValue(m_NetDevicesTable[i]->GetPhy()));
            malicioushelper.SetAttribute("Mac", PointerValue(m_NetDevicesTable[i]->GetMac()));

            malicioushelper.SetAttribute("GPS_PRN0", PointerValue(m_GPS_device_Table_PRN0[i]));
            malicioushelper.SetAttribute("GPS_PRN1", PointerValue(m_GPS_device_Table_PRN1[i]));
            malicioushelper.SetAttribute("GPS_PRN2", PointerValue(m_GPS_device_Table_PRN2[i]));
            malicioushelper.SetAttribute("GPS_PRN3", PointerValue(m_GPS_device_Table_PRN3[i]));

            malicioushelper.SetAttribute("GPS_PRN4", PointerValue(m_GPS_device_Table_PRN4[i]));
            malicioushelper.SetAttribute("GPS_PRN5", PointerValue(m_GPS_device_Table_PRN5[i]));
            malicioushelper.SetAttribute("GPS_PRN6", PointerValue(m_GPS_device_Table_PRN6[i]));
            malicioushelper.SetAttribute("GPS_PRN7", PointerValue(m_GPS_device_Table_PRN7[i]));

            malicioushelper.SetAttribute("GPS_PRN8", PointerValue(m_GPS_device_Table_PRN8[i]));
            malicioushelper.SetAttribute("GPS_PRN9", PointerValue(m_GPS_device_Table_PRN9[i]));
            malicioushelper.SetAttribute("GPS_PRN10", PointerValue(m_GPS_device_Table_PRN10[i]));
            malicioushelper.SetAttribute("GPS_PRN11", PointerValue(m_GPS_device_Table_PRN11[i]));

            malicioushelper.SetAttribute("MyPRN", UintegerValue(authenticPRN[i % 12]));

            m_GPS_device_Table_PRN0[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN1[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN2[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN3[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN4[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN5[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN6[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN7[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN8[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN9[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN10[i]->GetPhy()->m_GPSstate = 0;
            m_GPS_device_Table_PRN11[i]->GetPhy()->m_GPSstate = 0;

            m_DroneApps = malicioushelper.Install(m_drones.Get(i));
        }
    }

    NS_LOG_INFO("bada fin InstallApps");
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************creation d'un seul uav**************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::AddOneUAV(uint32_t DroneType,
                           int MobilityType,
                           double x,
                           double y,
                           double z,
                           double spped)
{
    // il faut creer les noeud avant les satellite!!!!

    uint32_t authenticPRN[] = {
        4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
        2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
        3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
        549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
        28594359,   349684609,  9999122,    224443546};

    /********************************** affectation d'adresse pour one UAV
     * ***********************************************/

    m_numDrones = m_numDrones + 1;
    Ptr<Node> UAV = CreateObject<Node>();

    char* LrWPANadresse;
    LrWPANadresse = (char*)malloc(5 * sizeof(char));
    *LrWPANadresse = '0';
    *(LrWPANadresse + 1) = '0';
    *(LrWPANadresse + 2) = ':';
    *(LrWPANadresse + 3) = '0';
    *(LrWPANadresse + 4) = '0';

    int LastGlobalIndex = m_numSatellites + m_numDrones - 1; // numero de drone sur netanim
    int j = m_numDrones - 1; // numero effective de drone a partir du 0

    std::stringstream str1;
    str1 << (LastGlobalIndex);

    Ptr<LrWpanNetDevice> NewDev_i = CreateObject<LrWpanNetDevice>();

    if (LastGlobalIndex < 10)
    {
        LrWPANadresse[4] = str1.str()[0];
        NewDev_i->SetAddress(
            Mac16Address(LrWPANadresse)); // affecter l'adresse MAC16 pour la carte reseau i
        m_NetDevicesTable.push_back(NewDev_i);
    }
    else
    {
        *(LrWPANadresse + 4) = str1.str()[1];
        *(LrWPANadresse + 3) = str1.str()[0];

        NewDev_i->SetAddress(Mac16Address(LrWPANadresse));
        m_NetDevicesTable.push_back(NewDev_i);
    }

    m_drones.Add(UAV);

    /********************************mobility one uav selon la
     * demande***********************************************/

    // Tahar :: Setting Positions

    if (MobilityType == UAV_CONSTANTE_MOBILITY)
    {
        Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();
        NewDev_i->GetPhy()->SetMobility(mob0);
        mob0->SetPosition(Vector(x, y, z));

        // recuperer la mobility qu'on a deja affecter afin de la visualiser sur netanim
        Ptr<ConstantPositionMobilityModel>
            mm0; // sert a recuperer la mobility de l'uavi pour le deploimenet en netanim
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(UAV);
        mm0 = UAV->GetObject<ConstantPositionMobilityModel>();
        mm0->SetPosition(Vector(x, y, z));
    }

    if (MobilityType == UAV_RANDOM_WALK_MOBILITY)
    {
        MobilityHelper mobility1;
        mobility1.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                       "X",
                                       StringValue(UniformRange(0.0, 100.0)),
                                       "Y",
                                       StringValue(UniformRange(0.0, 100.0)),
                                       "Z",
                                       StringValue(UniformRange(500.0, 1000.0)));
        mobility1.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                   "Mode",
                                   StringValue("Time"),
                                   "Time",
                                   StringValue("0.5s"),
                                   "Speed",
                                   StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
                                   "Bounds",
                                   StringValue(WalkBounds(0.0, 100.0, 0.0, 100.0)));

        mobility1.Install(UAV);

        Ptr<RandomWalk2dMobilityModel> mob1 = CreateObject<RandomWalk2dMobilityModel>();
        mob1->SetAttribute("Mode", StringValue("Time"));
        mob1->SetAttribute("Time", StringValue("0.5s"));
        mob1->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"));
        mob1->SetAttribute("Bounds", StringValue(WalkBounds(0.0, 100.0, 0.0, 100.0)));

        NewDev_i->GetPhy()->SetMobility(mob1);
    }

    if (MobilityType == UAV_HERARCHICAL_MOBILITY)
    {
        MobilityHelper mobility;

        // le 1er type de mobility: constante position
        Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();

        // le 2eme type de mobility: Random walk 2d
        Ptr<RandomWalk2dMobilityModel> mob1 = CreateObject<RandomWalk2dMobilityModel>();
        mob1->SetAttribute("Mode", StringValue("Time"));
        mob1->SetAttribute("Time", StringValue("0.5s"));
        mob1->SetAttribute(
            "Speed",
            StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(spped) + "]"));
        mob1->SetAttribute("Bounds", StringValue(WalkBounds(0.0, 20000.0, 0.0, 20000.0)));
        m_RandomMobilityTable.push_back(mob1);
        // affecter les 2 mobility
        mobility.SetMobilityModel("ns3::HierarchicalMobilityModel",
                                  "Child",
                                  PointerValue(mob1),
                                  "Parent",
                                  PointerValue());

        mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                      "X",
                                      StringValue(UniformRange(0.0, 20000.0)),
                                      "Y",
                                      StringValue(UniformRange(0.0, 20000.0)),
                                      "Z",
                                      StringValue(UniformRange(500.0, 1000.0)));

        // install mobility pour netanim
        mobility.Install(UAV);

        // install mobility pour les devices lrwpan
        NewDev_i->GetPhy()->SetMobility(mob1);

        // allocation netanim

        Ptr<HierarchicalMobilityModel> Mob = UAV->GetObject<HierarchicalMobilityModel>();
        Mob->SetPosition(Vector(x, y, z));

        // allocation lrwpan
        mob1->SetPosition(Vector(x, y, z));
    }

    /**********************************affectation du channel of data
     * OneUav**********************************************/

    LrWpanSpectrumValueHelper svh;

    Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity(0.0, channelNumber);

    // affectation de la puisance + channnel
    NewDev_i->SetChannel(channel);
    NewDev_i->GetPhy()->SetTxPowerSpectralDensity(psd);

    UAV->AddLrWpanNetDevice(NewDev_i);
    UAV->AddDevice(NewDev_i);

    /****************************installation d'un recepteur GPS pour one
     * UAV***********************************************/

    char GPSadresse[5] = {'7', '7', ':', '7', '7'};

    // creation de  la carte reseau pour chaque drone (adresse MAC sur 16 bits / 2 octets de 00:00
    // jusqua FF:FF)
    //  + stockage des adresses MAC16 dans un vecteur de pointeur   m_NetDevicesTable de
    //  LrWpanNetDevice
    Ptr<LrWpanNetDevice> GPSdev_PRN0;
    Ptr<LrWpanNetDevice> GPSdev_PRN1;
    Ptr<LrWpanNetDevice> GPSdev_PRN2;
    Ptr<LrWpanNetDevice> GPSdev_PRN3;
    Ptr<LrWpanNetDevice> GPSdev_PRN4;
    Ptr<LrWpanNetDevice> GPSdev_PRN5;
    Ptr<LrWpanNetDevice> GPSdev_PRN6;
    Ptr<LrWpanNetDevice> GPSdev_PRN7;
    Ptr<LrWpanNetDevice> GPSdev_PRN8;
    Ptr<LrWpanNetDevice> GPSdev_PRN9;
    Ptr<LrWpanNetDevice> GPSdev_PRN10;
    Ptr<LrWpanNetDevice> GPSdev_PRN11;

    GPSdev_PRN0 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN1 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN2 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN3 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN4 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN5 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN6 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN7 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN8 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN9 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN10 = CreateObject<LrWpanNetDevice>();
    GPSdev_PRN11 = CreateObject<LrWpanNetDevice>();

    GPSdev_PRN0->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN1->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN2->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN3->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i

    GPSdev_PRN4->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN5->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN6->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN7->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i

    GPSdev_PRN8->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN9->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN10->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i
    GPSdev_PRN11->SetAddress(
        Mac16Address(GPSadresse)); // affecter l'adresse MAC16 pour la carte reseau i

    m_GPS_device_Table_PRN0.push_back(GPSdev_PRN0);
    m_GPS_device_Table_PRN1.push_back(GPSdev_PRN1);
    m_GPS_device_Table_PRN2.push_back(GPSdev_PRN2);
    m_GPS_device_Table_PRN3.push_back(GPSdev_PRN3);

    m_GPS_device_Table_PRN4.push_back(GPSdev_PRN4);
    m_GPS_device_Table_PRN5.push_back(GPSdev_PRN5);
    m_GPS_device_Table_PRN6.push_back(GPSdev_PRN6);
    m_GPS_device_Table_PRN7.push_back(GPSdev_PRN7);

    m_GPS_device_Table_PRN8.push_back(GPSdev_PRN8);
    m_GPS_device_Table_PRN9.push_back(GPSdev_PRN9);
    m_GPS_device_Table_PRN10.push_back(GPSdev_PRN10);
    m_GPS_device_Table_PRN11.push_back(GPSdev_PRN11);

    GPSdev_PRN0->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN1->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN2->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN3->GetPhy()->m_isSatellite = 1;

    GPSdev_PRN4->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN5->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN6->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN7->GetPhy()->m_isSatellite = 1;

    GPSdev_PRN8->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN9->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN10->GetPhy()->m_isSatellite = 1;
    GPSdev_PRN11->GetPhy()->m_isSatellite = 1;

    GPSdev_PRN0->GetPhy()->m_PrnChannel = authenticPRN[0];
    GPSdev_PRN1->GetPhy()->m_PrnChannel = authenticPRN[1];
    GPSdev_PRN2->GetPhy()->m_PrnChannel = authenticPRN[2];
    GPSdev_PRN3->GetPhy()->m_PrnChannel = authenticPRN[3];

    GPSdev_PRN4->GetPhy()->m_PrnChannel = authenticPRN[4];
    GPSdev_PRN5->GetPhy()->m_PrnChannel = authenticPRN[5];
    GPSdev_PRN6->GetPhy()->m_PrnChannel = authenticPRN[6];
    GPSdev_PRN7->GetPhy()->m_PrnChannel = authenticPRN[7];

    GPSdev_PRN8->GetPhy()->m_PrnChannel = authenticPRN[8];
    GPSdev_PRN9->GetPhy()->m_PrnChannel = authenticPRN[9];
    GPSdev_PRN10->GetPhy()->m_PrnChannel = authenticPRN[10];
    GPSdev_PRN11->GetPhy()->m_PrnChannel = authenticPRN[11];

    /***********************************affectation d'un GPS channel pour one
     * uav***********************************************/

    LrWpanSpectrumValueHelper GPSsvh;
    Ptr<SpectrumValue> GPSpsd = GPSsvh.CreateTxPowerSpectralDensity(57.0, GPSchannelNumber);

    // affectation de la puisance + channnel
    GPSdev_PRN0->SetChannel(GPSchannel);
    GPSdev_PRN1->SetChannel(GPSchannel);
    GPSdev_PRN2->SetChannel(GPSchannel);
    GPSdev_PRN3->SetChannel(GPSchannel);
    GPSdev_PRN4->SetChannel(GPSchannel);
    GPSdev_PRN5->SetChannel(GPSchannel);
    GPSdev_PRN6->SetChannel(GPSchannel);
    GPSdev_PRN7->SetChannel(GPSchannel);
    GPSdev_PRN8->SetChannel(GPSchannel);
    GPSdev_PRN9->SetChannel(GPSchannel);
    GPSdev_PRN10->SetChannel(GPSchannel);
    GPSdev_PRN11->SetChannel(GPSchannel);

    GPSdev_PRN0->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN1->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN2->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN3->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN4->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN5->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN6->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN7->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN8->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN9->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN10->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);
    GPSdev_PRN11->GetPhy()->SetTxPowerSpectralDensity(GPSpsd);

    UAV->AddLrWpanNetDevice(GPSdev_PRN0);
    UAV->AddLrWpanNetDevice(GPSdev_PRN1);
    UAV->AddLrWpanNetDevice(GPSdev_PRN2);
    UAV->AddLrWpanNetDevice(GPSdev_PRN3);
    UAV->AddLrWpanNetDevice(GPSdev_PRN4);
    UAV->AddLrWpanNetDevice(GPSdev_PRN5);
    UAV->AddLrWpanNetDevice(GPSdev_PRN6);
    UAV->AddLrWpanNetDevice(GPSdev_PRN7);
    UAV->AddLrWpanNetDevice(GPSdev_PRN8);
    UAV->AddLrWpanNetDevice(GPSdev_PRN9);
    UAV->AddLrWpanNetDevice(GPSdev_PRN10);
    UAV->AddLrWpanNetDevice(GPSdev_PRN11);

    UAV->AddDevice(GPSdev_PRN0);
    UAV->AddDevice(GPSdev_PRN1);
    UAV->AddDevice(GPSdev_PRN2);
    UAV->AddDevice(GPSdev_PRN3);
    UAV->AddDevice(GPSdev_PRN4);
    UAV->AddDevice(GPSdev_PRN5);
    UAV->AddDevice(GPSdev_PRN6);
    UAV->AddDevice(GPSdev_PRN7);
    UAV->AddDevice(GPSdev_PRN8);
    UAV->AddDevice(GPSdev_PRN9);
    UAV->AddDevice(GPSdev_PRN10);
    UAV->AddDevice(GPSdev_PRN11);

    /**********************************l'installation de l'app pour one
     * uav**********************************************/

    if (DroneType == UAV_GOOD)
    {
        NormalDroneHelper normalHelper;

        normalHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
        normalHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));

        normalHelper.SetAttribute("Mob", PointerValue(m_RandomMobilityTable[j]));
        normalHelper.SetAttribute("Phy", PointerValue(NewDev_i->GetPhy()));
        normalHelper.SetAttribute("Mac", PointerValue(NewDev_i->GetMac()));

        normalHelper.SetAttribute("GPS_PRN0", PointerValue(GPSdev_PRN0));
        normalHelper.SetAttribute("GPS_PRN1", PointerValue(GPSdev_PRN1));
        normalHelper.SetAttribute("GPS_PRN2", PointerValue(GPSdev_PRN2));
        normalHelper.SetAttribute("GPS_PRN3", PointerValue(GPSdev_PRN3));

        normalHelper.SetAttribute("GPS_PRN4", PointerValue(GPSdev_PRN4));
        normalHelper.SetAttribute("GPS_PRN5", PointerValue(GPSdev_PRN5));
        normalHelper.SetAttribute("GPS_PRN6", PointerValue(GPSdev_PRN6));
        normalHelper.SetAttribute("GPS_PRN7", PointerValue(GPSdev_PRN7));

        normalHelper.SetAttribute("GPS_PRN8", PointerValue(GPSdev_PRN8));
        normalHelper.SetAttribute("GPS_PRN9", PointerValue(GPSdev_PRN9));
        normalHelper.SetAttribute("GPS_PRN10", PointerValue(GPSdev_PRN10));
        normalHelper.SetAttribute("GPS_PRN11", PointerValue(GPSdev_PRN11));

        // normalHelper.SetAttribute ("GPS", PointerValue( m_GPS_device_Table_PRN0[i]->GetPhy ()));

        m_DroneApps = normalHelper.Install(UAV);
    }
    if (DroneType == UAV_BAD)
    {
        // affecter la'pplication malveillante au drone n0
        MaliciousDroneHelper malicioushelper;

        malicioushelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
        malicioushelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));

        malicioushelper.SetAttribute("Mob", PointerValue(m_RandomMobilityTable[j]));
        malicioushelper.SetAttribute("Phy", PointerValue(NewDev_i->GetPhy()));
        malicioushelper.SetAttribute("Mac", PointerValue(NewDev_i->GetMac()));

        malicioushelper.SetAttribute("GPS_PRN0", PointerValue(GPSdev_PRN0));
        malicioushelper.SetAttribute("GPS_PRN1", PointerValue(GPSdev_PRN1));
        malicioushelper.SetAttribute("GPS_PRN2", PointerValue(GPSdev_PRN2));
        malicioushelper.SetAttribute("GPS_PRN3", PointerValue(GPSdev_PRN3));

        malicioushelper.SetAttribute("GPS_PRN4", PointerValue(GPSdev_PRN4));
        malicioushelper.SetAttribute("GPS_PRN5", PointerValue(GPSdev_PRN5));
        malicioushelper.SetAttribute("GPS_PRN6", PointerValue(GPSdev_PRN6));
        malicioushelper.SetAttribute("GPS_PRN7", PointerValue(GPSdev_PRN7));

        malicioushelper.SetAttribute("GPS_PRN8", PointerValue(GPSdev_PRN8));
        malicioushelper.SetAttribute("GPS_PRN9", PointerValue(GPSdev_PRN9));
        malicioushelper.SetAttribute("GPS_PRN10", PointerValue(GPSdev_PRN10));
        malicioushelper.SetAttribute("GPS_PRN11", PointerValue(GPSdev_PRN11));

        m_DroneApps = malicioushelper.Install(UAV);
    }
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/*********************************** satellites ***********************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::CreateNSatellite(int NbrSatellite, double TxPower, double StartTime)
{
    // TxPower= 57.0
    // uint32_t authenticPRN[]= {13234425, 24924213,
    // 8432419,1040493,139933,141103037,170400210,20330,666,209572295,5938495,692234,960990234,546345,9953,77322430,92934875,923840985,90283402,54324234,542489234,549024,1212,44359,70398234,3239,834873258,60324892,28594359,349684609,9999122,224443546};
    uint32_t authenticPRN[] = {
        4294967295, 2863311530, 3435973836, 2576980377, 4042322160, 2779096485, 3284386755,
        2526451350, 4278255360, 2857740885, 3425946675, 2573637990, 4027576335, 2774181210,
        3275539260, 2523502185, 92934875,   923840985,  90283402,   54324234,   542489234,
        549024,     1212,       44359,      70398234,   3239,       834873258,  60324892,
        28594359,   349684609,  9999122,    224443546};

    Vector SatellitePosition[] = {Vector(2, 2, 20100000),         Vector(1, 10000, 20100000),
                                  Vector(10000, 10000, 20100000), Vector(10000, 1, 20100000),
                                  Vector(1, 20000, 20100000),     Vector(10000, 20000, 20100000),
                                  Vector(20000, 20000, 20100000), Vector(20000, 10000, 20100000),
                                  Vector(20000, 0, 20100000),     Vector(1, 30000, 20100000),
                                  Vector(10000, 30000, 20100000), Vector(20000, 30000, 20100000),
                                  Vector(30000, 30000, 20100000), Vector(30000, 20000, 20100000),
                                  Vector(30000, 10000, 20100000), Vector(30000, 1, 20100000),
                                  Vector(1, 40000, 20100000),     Vector(10000, 40000, 20100000),
                                  Vector(20000, 40000, 20100000), Vector(30000, 40000, 20100000),
                                  Vector(40000, 40000, 20100000), Vector(40000, 30000, 20100000),
                                  Vector(40000, 20000, 20100000), Vector(40000, 10000, 20100000),
                                  Vector(40000, 1, 20100000),     Vector(1, 50000, 20100000),
                                  Vector(10000, 50000, 20100000), Vector(20000, 50000, 20100000),
                                  Vector(30000, 50000, 20100000), Vector(40000, 50000, 20100000),
                                  Vector(50000, 50000, 20100000), Vector(50000, 40000, 20100000)};
    for (int i = 0; i < NbrSatellite; i++)
    {
        // Tahar : Satellite positions
        std::cout << "Satellite " << i << " position : x = " << SatellitePosition[i % 32].x << std::endl;
        std::cout << "Satellite " << i << " position : x = " << SatellitePosition[i % 32].y << std::endl;
        std::cout << "Satellite " << i << " position : x = " << SatellitePosition[i % 32].z << std::endl;
        CreateOneSatellite(SatellitePosition[i % 32].x,
                           SatellitePosition[i % 32].y,
                           SatellitePosition[i % 32].z,
                           authenticPRN[m_numSatellites % 12],
                           TxPower,
                           StartTime); // un seule satellite
    }
}

void
DroneExperiment::Create12Satellite(double TxPower)
{
    NS_LOG_INFO("bada debut create sattelite");

    m_numSatellites = 12;

    // cretion des 32 noeuds satellites
    m_satellites.Create(m_numSatellites);

    // affectation des adresses   01:55 ==> 32:55
    char adresse[5] = {'0', '0', ':', '0', '0'};

    for (uint32_t i = (0 + m_numDrones); i < (m_numSatellites + m_numDrones); i++)
    {
        std::stringstream str;
        str << (i);
        Ptr<LrWpanNetDevice> dev_sattelite = CreateObject<LrWpanNetDevice>();
        dev_sattelite->GetPhy()->m_isSatellite = 1;

        if (i < 10)
        {
            adresse[1] = str.str()[0];
            dev_sattelite->SetAddress(
                Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
            m_NetDevicesSatelliteTable.push_back(dev_sattelite);
        }
        else
        {
            adresse[1] = str.str()[1];
            adresse[0] = str.str()[0];

            dev_sattelite->SetAddress(Mac16Address(adresse));
            m_NetDevicesSatelliteTable.push_back(dev_sattelite);
        }

        dev_sattelite->GetPhy()
            ->DisableGPS(); // disable satellite to receive signal from other satellite
    }

    // affecter la mobility de chaque satellite (cote Lrwpan)

    double echelle = 1;

    // satellite1 :
    Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[0]->GetPhy()->SetMobility(mob0);
    mob0->SetPosition(Vector(echelle * 2, echelle * 2, 20200000));

    // satellite2 :
    Ptr<ConstantPositionMobilityModel> mob1 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[1]->GetPhy()->SetMobility(mob1);
    mob1->SetPosition(Vector(echelle * 2, echelle * 10000, 20200000));

    // satellite3 :
    Ptr<ConstantPositionMobilityModel> mob2 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[2]->GetPhy()->SetMobility(mob2);
    mob2->SetPosition(Vector(echelle * 10000, echelle * 2, 20200000));

    // satellite4 :
    Ptr<ConstantPositionMobilityModel> mob3 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[3]->GetPhy()->SetMobility(mob3);
    mob3->SetPosition(Vector(echelle * 10000, echelle * 10000, 20200000));

    // satellite5 :
    Ptr<ConstantPositionMobilityModel> mob4 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[4]->GetPhy()->SetMobility(mob4);
    mob4->SetPosition(Vector(echelle * 10000, echelle * 5000, 20200000));

    // satellite6 :
    Ptr<ConstantPositionMobilityModel> mob5 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[5]->GetPhy()->SetMobility(mob5);
    mob5->SetPosition(Vector(echelle * 2, echelle * 5000, 20200000));

    // satellite7 :
    Ptr<ConstantPositionMobilityModel> mob6 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[6]->GetPhy()->SetMobility(mob6);
    mob6->SetPosition(Vector(echelle * 5000, echelle * 2, 20200000));

    // satellite8 :
    Ptr<ConstantPositionMobilityModel> mob7 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[7]->GetPhy()->SetMobility(mob7);
    mob7->SetPosition(Vector(echelle * 5000, echelle * 5000, 20200000));

    // satellite9 :
    Ptr<ConstantPositionMobilityModel> mob8 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[8]->GetPhy()->SetMobility(mob8);
    mob8->SetPosition(Vector(echelle * 5000, echelle * 10000, 20200000));

    // satellite10 :
    Ptr<ConstantPositionMobilityModel> mob9 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[9]->GetPhy()->SetMobility(mob9);
    mob9->SetPosition(Vector(echelle * 7500, echelle * 7500, 20200000));

    // satellite11:
    Ptr<ConstantPositionMobilityModel> mob10 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[10]->GetPhy()->SetMobility(mob10);
    mob10->SetPosition(Vector(echelle * 2500, echelle * 7000, 20200000));

    // satellite12 :
    Ptr<ConstantPositionMobilityModel> mob11 = CreateObject<ConstantPositionMobilityModel>();

    m_NetDevicesSatelliteTable[11]->GetPhy()->SetMobility(mob11);
    mob11->SetPosition(Vector(echelle * 7500, echelle * 2500, 20200000));

    // recuperer la mobility qu'on a deja affecter afin de la visualiser sur netanim

    Ptr<ConstantPositionMobilityModel>
        mm0; // sert a recuperer la mobility de l'uavi pour le deploimenet en netanim
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // satellite1:
    mobility.Install(m_satellites.Get(0));
    mm0 = m_satellites.Get(0)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 2, echelle * 2, 20200000));

    // satellite2:
    mobility.Install(m_satellites.Get(1));
    mm0 = m_satellites.Get(1)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 2, echelle * 10000, 20200000));

    // satellite3:
    mobility.Install(m_satellites.Get(2));
    mm0 = m_satellites.Get(2)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 10000, echelle * 2, 20200000));

    // satellite4:
    mobility.Install(m_satellites.Get(3));
    mm0 = m_satellites.Get(3)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 10000, echelle * 10000, 20200000));

    // satellite5:
    mobility.Install(m_satellites.Get(4));
    mm0 = m_satellites.Get(4)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 10000, echelle * 5000, 20200000));

    // satellite6:
    mobility.Install(m_satellites.Get(5));
    mm0 = m_satellites.Get(5)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 2, echelle * 5000, 20200000));

    // satellite7:
    mobility.Install(m_satellites.Get(6));
    mm0 = m_satellites.Get(6)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 5000, echelle * 2, 20200000));

    // satellite8:
    mobility.Install(m_satellites.Get(7));
    mm0 = m_satellites.Get(7)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 5000, echelle * 5000, 20200000));

    // satellite9:
    mobility.Install(m_satellites.Get(8));
    mm0 = m_satellites.Get(8)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 5000, echelle * 10000, 20200000));

    // satellite10:
    mobility.Install(m_satellites.Get(9));
    mm0 = m_satellites.Get(9)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 7500, echelle * 7500, 20200000));

    // satellite11:
    mobility.Install(m_satellites.Get(10));
    mm0 = m_satellites.Get(10)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 2500, echelle * 7500, 20200000));

    // satellite12:
    mobility.Install(m_satellites.Get(11));
    mm0 = m_satellites.Get(11)->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(echelle * 7500, echelle * 2000, 20200000));

    // transmitedPower = 57.0
    LrWpanSpectrumValueHelper svh;
    Ptr<SpectrumValue> psd =
        svh.CreateTxPowerSpectralDensity(TxPower, GPSchannelNumber); // 26.8dbw = 57dbm = 479w

    // affectation de la puisance + channnel
    for (uint32_t i = 0; i < m_numSatellites; i++)
    {
        m_NetDevicesSatelliteTable[i]->SetChannel(GPSchannel);
        m_NetDevicesSatelliteTable[i]->GetPhy()->SetTxPowerSpectralDensity(psd);
    }

    // affecter les device
    for (uint32_t i = 0; i < m_numSatellites; i++)
    {
        m_satellites.Get(i)->AddLrWpanNetDevice(m_NetDevicesSatelliteTable[i]);
        m_satellites.Get(i)->AddDevice(m_NetDevicesSatelliteTable[i]);
    }

    // affecter l'application
    for (uint32_t i = 0; i < m_numSatellites; i++)
    {
        SatelliteHelper satelliteHelper;

        satelliteHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
        satelliteHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));
        satelliteHelper.SetAttribute("TxPower", DoubleValue(TxPower));
        // satelliteHelper.SetAttribute ("Mob", PointerValue( m_RandomMobilityTable[i]));
        satelliteHelper.SetAttribute("Phy", PointerValue(m_NetDevicesSatelliteTable[i]->GetPhy()));
        satelliteHelper.SetAttribute("Mac", PointerValue(m_NetDevicesSatelliteTable[i]->GetMac()));

        // normalHelper.SetAttribute ("GPS", PointerValue( m_GPS_device_Table_PRN0[i]->GetPhy ()));

        m_SatelliteApps = satelliteHelper.Install(m_satellites.Get(i));
    }

    NS_LOG_INFO("bada fin create satellite");
}

/**********************************************************************************************/
/**********************************************************************************************/
/**********************************************************************************************/

void
DroneExperiment::CreateOneSatellite(double x,
                                    double y,
                                    double z,
                                    uint32_t PRN,
                                    double TxPower,
                                    double StartTime)
{
    NS_LOG_INFO("bada debut create one sattelite");
    std::cout << " ----------------------------------------- debut "
                 "CreateOneSatellite------------------------------- \n";

    m_numSatellites = m_numSatellites + 1;

    Ptr<Node> satellite = CreateObject<Node>();
    char adresse[5] = {'0', '0', ':', '0', '0'};

    int i = m_numSatellites + m_numDrones - 1;
    int j = m_numSatellites - 1;

    std::cout << " i= : " << i << "\n";

    // affectation des adresses   01:55 ==> 32:55

    std::stringstream str;
    str << (i);
    Ptr<LrWpanNetDevice> dev_sattelite = CreateObject<LrWpanNetDevice>();
    dev_sattelite->GetPhy()->m_isSatellite = 1;
    dev_sattelite->GetPhy()->DisableGPS();

    if (i < 10)
    {
        adresse[1] = str.str()[0];
        dev_sattelite->SetAddress(
            Mac16Address(adresse)); // affecter l'adresse MAC16 pour la carte reseau i
        m_NetDevicesSatelliteTable.push_back(dev_sattelite);
    }
    else
    {
        adresse[1] = str.str()[1];
        adresse[0] = str.str()[0];

        dev_sattelite->SetAddress(Mac16Address(adresse));
        m_NetDevicesSatelliteTable.push_back(dev_sattelite);
    }
    Mac16Address Address = dev_sattelite->GetMac()->GetShortAddress();
    std::cout << "Full Address [" << dev_sattelite->GetMac() << "] \n\n";
    std::cout << "Full Address [" << dev_sattelite->GetMac()->GetExtendedAddress() << "] \n\n";
    std::cout << "			je suis le satellite [" << Address << "] \n\n";

    std::cout << " ----------------------------------------- adresse  c bn \n";

    Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();

    dev_sattelite->GetPhy()->SetMobility(mob0);
    mob0->SetPosition(Vector(x, y, z));

    Ptr<ConstantPositionMobilityModel>
        mm0; // sert a recuperer la mobility de l'uavi pour le deploimenet en netanim
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // satellite1:
    mobility.Install(satellite);
    mm0 = satellite->GetObject<ConstantPositionMobilityModel>();
    mm0->SetPosition(Vector(x, y, z));

    std::cout << " --------------------------------------mobility  c bn \n";

    LrWpanSpectrumValueHelper svh;
    Ptr<SpectrumValue> psd =
        svh.CreateTxPowerSpectralDensity(TxPower, GPSchannelNumber); // 26.8dbw = 57dbm = 479w

    // affectation de la puisance + channnel

    std::cout << " j= : " << j << "\n";

    dev_sattelite->SetChannel(GPSchannel);
    dev_sattelite->GetPhy()->SetTxPowerSpectralDensity(psd);
    std::cout << " --------------------------------------channel  c bn \n";

    // affecter les device

    satellite->AddLrWpanNetDevice(dev_sattelite);
    satellite->AddDevice(dev_sattelite);

    std::cout << " --------------------------------------device  c bn \n";

    Simulator::Schedule(Seconds(StartTime),
                        &DroneExperiment::startSatelliteApp,
                        this,
                        satellite,
                        dev_sattelite,
                        TxPower,
                        PRN);

    NS_LOG_INFO("bada fin create one satellite");
}

void
DroneExperiment::startSatelliteApp(Ptr<Node> satellite,
                                   Ptr<LrWpanNetDevice> dev_sattelite,
                                   double TxPower,
                                   uint32_t PRN)
{
    SatelliteHelper satelliteHelper;

    satelliteHelper.SetAttribute("PacketFrequency", TimeValue(Seconds(m_DataUavFrequency)));
    satelliteHelper.SetAttribute("PacketSize", UintegerValue(m_PacketSize));
    satelliteHelper.SetAttribute("TxPower", DoubleValue(TxPower));
    satelliteHelper.SetAttribute("PRN", UintegerValue(PRN));

    // satelliteHelper.SetAttribute ("Mob", PointerValue( m_RandomMobilityTable[i]));
    satelliteHelper.SetAttribute("Phy", PointerValue(dev_sattelite->GetPhy()));
    satelliteHelper.SetAttribute("Mac", PointerValue(dev_sattelite->GetMac()));

    m_SatelliteApps = satelliteHelper.Install(satellite);
    // m_SatelliteApps.Start(Seconds (20.0));
    // m_SatelliteApps.Stop(Seconds (30.0));

    m_satellites.Add(satellite);

    std::cout << " ----------------------------------------- fin "
                 "CreateOneSatellite------------------------------- \n";
}

/***************************************************************************/
/***********************************outils*********************************/
/***************************************************************************/

Vector
DroneExperiment::GetNext(uint32_t m_current,
                         int m_layoutType,
                         double m_xMin,
                         double m_yMin,
                         double m_deltaX,
                         double m_deltaY,
                         uint32_t NbrMaxParLigne)
{
    double x = 0.0, y = 0.0;
    switch (m_layoutType)
    {
    case 1:
        x = m_xMin + m_deltaX * (m_current % NbrMaxParLigne);
        y = m_yMin + m_deltaY * (m_current / NbrMaxParLigne);
        break;
    case 2:
        x = m_xMin + m_deltaX * (m_current / NbrMaxParLigne);
        y = m_yMin + m_deltaY * (m_current % NbrMaxParLigne);
        break;
    }
    std::cout << "			    < x== " << x << "      >> \n";
    std::cout << "			    < y== " << y << "      >> \n";

    return Vector(x, y, 1000.0);
}

std::string
DroneExperiment::UniformRange(uint32_t min, uint32_t max)
{
    NS_LOG_INFO("bada debut + fin UniformRange");

    std::cout << "MIN : " << min << std::endl;
    std::cout << "MAX : " << max << std::endl;

    return "ns3::UniformRandomVariable[Min=" + std::to_string(min) + "|Max=" + std::to_string(max) +
           "]";
}

std::string
DroneExperiment::WalkBounds(uint32_t minX, uint32_t maxX, uint32_t minY, uint32_t maxY)
{
    NS_LOG_INFO("bada debut + fin WalkBounds");

    return std::to_string(minX) + "|" + std::to_string(maxX) + "|" + std::to_string(minY) + "|" +
           std::to_string(maxY);
}

double
DroneExperiment::Percentage(int amount, int outOf)
{
    NS_LOG_INFO("bada debut + fin Percentage");

    return (amount / ((double)outOf)) * 100.0;
}

void
DroneExperiment::RecordStats()
{
    NS_LOG_INFO("bada debut  RecordStats");

    if (m_packetsSent != 0)
    {
        int diffSent = m_packetsSent - cp_packetsSent;
        int diffReceived = m_SpoofingSatellitePower - cp_packetsReceived;
        double packetLossRate = Percentage(diffSent - diffReceived, diffSent);
        m_cn0VersusAbsolutPower.Add((int)Simulator::Now().GetSeconds(), packetLossRate);
        cp_packetsSent = m_packetsSent;
        cp_packetsReceived = m_SpoofingSatellitePower;
    }
    if (m_dataReceived != 0)
    {
        m_authenticSignal.Add((int)Simulator::Now().GetSeconds(),
                              (m_dataReceived - cp_dataReceived) / 2.0);
        cp_dataReceived = m_dataReceived;
    }
    Simulator::Schedule(Seconds(2.0), &DroneExperiment::RecordStats, this);

    NS_LOG_INFO("bada fin  RecordStats");
}

void
DroneExperiment::ConnectCallbacks()
{
    NS_LOG_INFO("bada debut ConnectCallbacks");

    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/Rx",
                                  MakeCallback(&DroneExperiment::Cn0VersusTSP, this));
    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/RxAlert",
                                  MakeCallback(&DroneExperiment::Alert, this));
    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::NormalDroneApp/RxDetection",
                                  MakeCallback(&DroneExperiment::Detection, this));

    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::Malicious/Rx",
                                  MakeCallback(&DroneExperiment::Cn0VersusTSP, this));

    NS_LOG_INFO("bada fin ConnectCallbacks");
}

void
DroneExperiment::Cn0VersusTSP(double ReceivingSatellitepower, double CN0)
{
    std::cout << " ---------------------------debut "
                 "-----Cn0VersusTSP--------------------------------------- \n";

    // NS_LOG_INFO ("bada debut Cn0VersusTSP");

    // NS_LOG_INFO (std::to_string (packet->GetUid()));
    m_SpoofingSatellitePower += 1;

    double txSpoofingPower = m_SpoofingSatellitePower - 30;
    double rxSpoofingPower = txSpoofingPower - 182.4 - 2.0;

    std::cout << "			    < satellite power== " << ReceivingSatellitepower << "      >> \n";
    std::cout << "			    < c/n0== " << CN0 << "      >> \n";
    std::cout << "			    < rxSpoofingPower== " << rxSpoofingPower << "      >> \n";

    m_cn0VersusAbsolutPower.Add(rxSpoofingPower, CN0);

    // m_dataReceived += packet->GetSize ();
    //  m_cn0VersusAbsolutPower.Add (Simulator::Now(),Percentage (m_dataReceived,m_dataSent));
    // NS_LOG_INFO ("bada fin Cn0VersusTSP");
    std::cout << " ---------------------------fin "
                 "-----Cn0VersusTSP--------------------------------------- \n";
}

void
DroneExperiment::Alert(double ReceivingSatellitepower, double CN0, double index)
{
    std::cout
        << " ---------------------------debut -----Alert--------------------------------------- \n";

    if (index == 10)
    {
        m_True_Positive++;
        std::cout << "	    	m_True_Positive++;     >> \n";
    }

    if (index == 15)
    {
        m_False_Positive++;
        std::cout << "	    	m_False_Positive++;     >> \n";
    }

    if (index == 20)
    {
        m_True_Negative++;
        std::cout << "	    		m_True_Negative++;     >> \n";
    }

    if (index == 25)
    {
        m_False_Negative++;
        std::cout << "	    			m_False_Negative++;    >> \n";
    }

    std::cout
        << " ---------------------------fin -----Alert--------------------------------------- \n";
}

void
DroneExperiment::Detection(double ReceivingSatellitepower, double CN0, double index)
{
    if (m_method == Burglary)
    {
        m_True_Positive++;
        m_False_Positive--;
    }
}

std::vector<Gnuplot2dDataset>
DroneExperiment::Run(MethodName method, bool Dynamic, bool onStateChange)
{
    NS_LOG_INFO("bada debut Run");

    RngSeedManager::ResetStreamIndex();
    m_method = method;
    m_Dynamic = Dynamic;
    m_onStateChange = onStateChange;

    SetDefaults();

    /**********************************************************************************************/
    /**************************** initialisation du nombre d'UAV***********************************/
    /**********************************************************************************************/

    int NbrUAV = 100; // 100;
    int debut = 0;

    limite G1;
    G1.MinX = 1.0;
    G1.MaxX = 50000.0;

    G1.MinY = 1.0;
    G1.MaxY = 100000.0;

    G1.MinZ = 1000.0;
    G1.MaxZ = 10000.0;

    G1.NbrOfUAV = 100; // 100;
    G1.Speed = 200.0;

    limite G2;
    G2.MinX = 10000.0;
    G2.MaxX = 20000.0;

    G2.MinY = 1.0;
    G2.MaxY = 10000.0;

    G2.MinZ = 1000.0;
    G2.MaxZ = 5000.0;

    G2.NbrOfUAV = 0;
    G2.Speed = 100.0;

    limite G3;
    G3.MinX = 10000.0;
    G3.MaxX = 20000.0;

    G3.MinY = 10000.0;
    G3.MaxY = 20000.0;

    G3.MinZ = 1000.0;
    G3.MaxZ = 4000.0;

    G3.NbrOfUAV = 0;
    G3.Speed = 200.0;

    /**********************************************************************************************/
    /*******************************creation des UAVs**********************************************/
    /**********************************************************************************************/

    std::cout << "# Tahar : creation des UAVs \n";

    CreateNodesLrWPAN(NbrUAV);
    AffectationDesAddress_LrWPAN(debut, NbrUAV);
    AffectationDesAddress_GPS_PRN1(debut, NbrUAV, 0);

    // InstallHierarchicalMobilityModelLrwpan(debut, NbrUAV, G1);  //
    // InstallHierarchicalMobilityLrwpanWithGridAllocotar();
    InstallRandomMobilityLrWPAN(debut, NbrUAV, G1, G2, G3);
    // InstallHierarchicalMobilityLrwpanWithGridAllocotar(debut,NbrUAV,G1,G2,G3);

    std::cout << "# Tahar : InitialisationChanelLrwpan() \n";

    InitialisationChanelLrwpan();

    std::cout << "# Tahar : InitialisationChanelGPS() \n";
    InitialisationChanelGPS();

    InstallChannelLrwpan(debut, NbrUAV);
    InstallChannelGPS(debut, NbrUAV);

    InstallDevicesLrwpan(debut, NbrUAV);
    InstallDevicesGPS(debut, NbrUAV);

    int debutMalicious = 25; // 5;
    int finMalicious = 35;   // 30;
    int debutWitness = finMalicious;
    int finWitness = NbrUAV;

    std::cout << "# Tahar : Install Apps : (Good) \n";
    InstallApps(0, debutMalicious, UAV_VICTIM);         // good drone (1)
    std::cout << "# Tahar : Install Apps : (Bad) \n";
    InstallApps(debutMalicious, finMalicious, UAV_BAD); // bad drone (2)

    std::cout << "# Tahar : Install Apps : (Good) \n";
    InstallApps(debutWitness, finWitness, UAV_GOOD); // good drone (1)

    // AddOneUAV(UAV_GOOD, UAV_HERARCHICAL_MOBILITY, 2000,300,1000,100.0);
    // AddOneUAV(UAV_GOOD, UAV_HERARCHICAL_MOBILITY, 400,1000,1000,50.0);

    // tjr apres l'installation d'application

    ConnectCallbacks();

    if ((method == Burglary) || (method == AbsolutePower) || (method == AbsolutePower_CN0))
    {
        // creer 8 satellites
        CreateNSatellite(7, 57.0, 0.0);
    }

    /**********************************************************************************************/
    /*********************** creation des 12 satellites authentique *******************************/
    /**********************************************************************************************/
    //   uint32_t PRN= 13234425;
    

    // 58.6 ==> -155.8
    if (method == OneSatellite)
    { // Max C/N0

        uint32_t FirstPRN = 4294967295;
        CreateOneSatellite(1, 1, 20100000, FirstPRN, 58.6, 0.0); // un seule satellite
    }

    if (method == AllSatellite)
    { // Min C/N0

        CreateNSatellite(12, 57.0, 0.0);
    }

    if (method == NormalCase)
    { //  C/N0 moyenne avec 7 satellite

        CreateNSatellite(7, 57.0, 0.0);
    }

    /**********************************************************************************************/
    /*************************** creation des spoofing satellites *********************************/
    /**********************************************************************************************/
    std::cout << "# Tahar : CREATE Spoof Satellites : \n";

    if (method == AuthenticWithSpoofing)
    {
        for (uint32_t i = 0; i < m_numDrones; i++)
        {
            m_GPS_device_Table_PRN0[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN1[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN2[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN3[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN4[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN5[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN6[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN7[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN8[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN9[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN10[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN11[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
        }

        CreateNSatellite(12, 58.6, 0.0);

        // 10 spoofing
        double power = m_SpoofingSatellitePower;
        double AppStartTime = 0.006;

        CreateNSatellite(30, power, AppStartTime);
    }

    if (method == Spoofing10 || method == Spoofing20 || method == Spoofing30)
    {
        for (uint32_t i = 0; i < m_numDrones; i++)
        {
            m_GPS_device_Table_PRN0[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN1[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN2[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN3[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN4[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN5[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN6[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN7[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN8[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN9[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN10[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
            m_GPS_device_Table_PRN11[i]->GetPhy()->SetSatelliteReceptionMode(FIFO);
        }

        // 7 authentic
        CreateNSatellite(12, 57.0, 0.006);

        // 10 spoofing
        double power = m_SpoofingSatellitePower;
        double AppStartTime = 0.00;
        CreateNSatellite(24, power, AppStartTime);
    }

    if (method == Spoofing20 || method == Spoofing30)
    {
        // ajouter 10 spoofing  ==> total = 20
        double power = m_SpoofingSatellitePower;
        double AppStartTime = 0.00;
        CreateNSatellite(10, power, AppStartTime);
    }

    if (method == Spoofing30)
    {
        // ajouter 10 spoofing  ==> total = 20

        double power = m_SpoofingSatellitePower;
        double AppStartTime = 0.00;
        CreateNSatellite(10, power, AppStartTime);
    }

    /**********************************************************************************************/
    /*************************execution de la simulation*******************************************/
    /**********************************************************************************************/

    AnimationInterface anim("Bada_Drone1.xml");
    anim.EnablePacketMetadata(true);

    Simulator::Stop(Seconds(5.0)); // 40.0 + 120.0
    Simulator::Run();
    Simulator::Destroy();

    NS_LOG_INFO("bada fin  Run");
    std::cout << "	m_False_Positive =" << m_False_Positive << " \n";
    std::cout << "	m_True_Positive =" << m_True_Positive << " \n";

    std::cout << "	m_False_Negative =" << m_False_Negative << " \n";
    std::cout << "	m_True_Negative =" << m_True_Negative << " \n";

    double success_rate = (m_True_Positive + m_True_Negative) /
                          (m_True_Positive + m_False_Positive + m_False_Negative + m_True_Negative);
    std::cout << "	succes Rate ="
              << (m_True_Positive + m_True_Negative) /
                     (m_True_Positive + m_False_Positive + m_False_Negative + m_True_Negative)
              << " \n";

    m_methodString = m_methodString + "( success rate=" + std::to_string(success_rate) + " )";

    double base = m_True_Positive + m_False_Positive + m_False_Negative + m_True_Negative;
    m_success_rate.Add(0, success_rate);
    m_success_rate.Add(1, 100 * m_True_Positive / base);
    m_success_rate.Add(3, 100 * m_True_Negative / base);
    m_success_rate.Add(2, 100 * m_False_Positive / base);
    m_success_rate.Add(4, 100 * m_False_Negative / base);

    return {m_success_rate, m_cn0VersusAbsolutPower};
}

/**********************************************************************************************/
/**********************************************************************************************/
/*____________________________________________MAIN____________________________________________*/
/**********************************************************************************************/
/**********************************************************************************************/

int
main(int argc, char* argv[])
{
    NS_LOG_INFO("bada debut  main");

    time_t currentTime;
    time(&currentTime); // Grab current time as seed if no seed is specified

    /**********************************************************************************************/
    /******************************creation des objets Gnuplot*************************************/
    /**********************************************************************************************/

    // objects for diagram
    std::ostringstream os;
    std::ofstream SpoofingOutPutStream_Threshold("SpoofingThreshold.plt");
    std::ofstream SpoofingOutPutStream_SuccessRate("Success_rate.plt");

    Gnuplot DiagramFileSpoofing_Threshold = Gnuplot("SpoofingThreshold.eps");
    Gnuplot DiagramFileSpoofing_Success_Rate = Gnuplot("Success_rate.eps");

    std::vector<Gnuplot2dDataset> DataSets_SpoofingThreshold = {};
    std::vector<Gnuplot2dDataset> DataSets_Success_Rate = {};

    DiagramFileSpoofing_Threshold.SetTitle("Figure1");
    DiagramFileSpoofing_Success_Rate.SetTitle("Figure2");

    // DiagramFileSpoofing_Success_Rate.SetTerminal ("postscript eps color enh
    // \"Times-BoldItalic\"");
    DiagramFileSpoofing_Threshold.SetTerminal("pdf");
    DiagramFileSpoofing_Success_Rate.SetTerminal("pdf");

    DiagramFileSpoofing_Threshold.SetLegend("Spoofing power (dBW)", "C/N0");
    DiagramFileSpoofing_Success_Rate.SetLegend("", "%");

    DiagramFileSpoofing_Threshold.SetExtra("set key font \"Verdana,8\" \n\
		  set xrange [-160:-140]\n\
    set yrange [35:55]\n\
    set grid\n\
    set style line 1 linewidth 5\n\
    set style increment user");

    DiagramFileSpoofing_Success_Rate.SetExtra(
        " set xtics(\"True_Positive\" 1 ,\"False_Positive\" 2, \"True_Negative\" 3,\"False_Negative\" 4) \n\
		  set xrange [0:5] \n\
		  set style data histogram \n\
		  set style fill solid  \n\
		  set style histogram clustered ");

    /**********************************************************************************************/
    /************************ initialisation de notre experience **********************************/
    /************************                  +                 **********************************/
    /********************** initialisation des lignes de commandes   ******************************/
    /**********************************************************************************************/

    LogComponentEnable("DroneAdhoc", LOG_LEVEL_INFO);

    std::string phyMode("OfdmRate9Mbps");

    uint32_t numDrones = 9;       // 30
    uint32_t maxX = 200;          // m
    uint32_t maxY = 200;          // m
    uint32_t collPacketSize = 24; // kilobytes (KB)
    double collFrequency = 1;     // seconds 0.5
    uint32_t imagePacketSize = 5; // megabytes (MB) 5
    double imageFrequency = 10.0; // seconds
    bool onStateChange = true;

    CommandLine cmd;

    cmd.AddValue("numDrones", "number of drones in network", numDrones);
    cmd.AddValue("maxY", "position of wall (top) for drone box (meters)", maxY);
    cmd.AddValue("maxX", "position of wall (right) for drone box (meters)", maxX);
    cmd.AddValue("collPacketSize", "collision detection packet size (KB)", collPacketSize);
    cmd.AddValue("collFrequency", "collision detection packet frequency (seconds)", collFrequency);
    cmd.AddValue("imagePacketSize", "image packet size (MB)", imagePacketSize);
    cmd.AddValue("imageFrequency", "image packet frequency (seconds)", imageFrequency);
    cmd.AddValue("onStateChange", "whether to adjust gains while idle.", onStateChange);
    cmd.AddValue("seed", "seed for the RngSeedManager", currentTime);

    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(currentTime);

    Gnuplot packetLoss = Gnuplot("packet-loss-rate.png");
    Gnuplot throughput = Gnuplot("data-throughput.png");

    Ptr<DroneExperiment> experiment;

    experiment = CreateObject<DroneExperiment>();
    experiment->SetAttribute("NumDrones", UintegerValue(numDrones));
    experiment->SetAttribute("MaxX", UintegerValue(maxX));
    experiment->SetAttribute("MaxY", UintegerValue(maxY));
    experiment->SetAttribute("CollisionPacketSize", UintegerValue(collPacketSize));
    experiment->SetAttribute("CollisionPacketFrequency", DoubleValue(collFrequency));
    experiment->SetAttribute("ImagePacketSize", UintegerValue(imagePacketSize * 1024 * 1024));
    experiment->SetAttribute("ImagePacketFrequency", DoubleValue(imageFrequency));
    experiment->SetAttribute("PHYmode", StringValue(phyMode));

    averages = std::to_string(numDrones);

    /**********************************************************************************************/
    /*********************** l'excution d'exeperience********************************************/
    /**********************************************************************************************/

    /**********************************************************************************************/
    /**************  affecter les resultats dans le fichier output stream**************************/
    /**********************************************************************************************/

    /*

      DataSets_SpoofingThreshold = experiment->Run (OneSatellite, true, onStateChange);
       DiagramFileSpoofing_Threshold.AddDataset(DataSets_SpoofingThreshold[1]);




       DataSets_SpoofingThreshold = experiment->Run (AllSatellite, true, onStateChange);
        DiagramFileSpoofing_Threshold.AddDataset(DataSets_SpoofingThreshold[1]);




        DataSets_SpoofingThreshold = experiment->Run (NormalCase, true, onStateChange);
            DiagramFileSpoofing_Threshold.AddDataset(DataSets_SpoofingThreshold[1]);



        DataSets_SpoofingThreshold = experiment->Run (AuthenticWithSpoofing, true, onStateChange);
         DiagramFileSpoofing_Threshold.AddDataset(DataSets_SpoofingThreshold[1]);



         DataSets_SpoofingThreshold = experiment->Run (Spoofing10, true, onStateChange);
          DiagramFileSpoofing_Threshold.AddDataset(DataSets_SpoofingThreshold[1]);


          DataSets_SpoofingThreshold = experiment->Run (Spoofing20, true, onStateChange);
               DiagramFileSpoofing_Threshold.AddDataset(DataSets_SpoofingThreshold[1]);


               DataSets_SpoofingThreshold = experiment->Run (Spoofing30, true, onStateChange);
               DiagramFileSpoofing_Threshold.AddDataset(DataSets_SpoofingThreshold[1]);


    */
    DataSets_Success_Rate = experiment->Run(Burglary, true, onStateChange);
    DataSets_Success_Rate[0].SetStyle(Gnuplot2dDataset::HISTEPS);
    DiagramFileSpoofing_Success_Rate.AddDataset(DataSets_Success_Rate[0]);
    /*
                DataSets_Success_Rate = experiment->Run (AbsolutePower, true, onStateChange);
                DataSets_Success_Rate[0].SetStyle(Gnuplot2dDataset::HISTEPS);
                DiagramFileSpoofing_Success_Rate.AddDataset(DataSets_Success_Rate[0]);


                DataSets_Success_Rate = experiment->Run (AbsolutePower_CN0, true, onStateChange);
                DataSets_Success_Rate[0].SetStyle(Gnuplot2dDataset::HISTEPS);
                DiagramFileSpoofing_Success_Rate.AddDataset(DataSets_Success_Rate[0]);
    */

    DiagramFileSpoofing_Success_Rate.GenerateOutput(SpoofingOutPutStream_SuccessRate);
    DiagramFileSpoofing_Threshold.GenerateOutput(SpoofingOutPutStream_Threshold);

    SpoofingOutPutStream_SuccessRate.close();
    /**********************************************************************************************/
    /**********************************************************************************************/
    /**********************************************************************************************/

    // packetLoss.AddDataset (DataSets_Success_Rate[0]);
    // throughput.AddDataset (DataSets_Success_Rate[1]);

    std::ofstream runAvgsFile;
    runAvgsFile.open("drone_avgs.csv", std::ios_base::app);
    runAvgsFile << averages + "\n";
    runAvgsFile.close();

    // packetLoss.GenerateOutput (std::cout);
    // throughput.GenerateOutput (std::cout);

    return 0;
}
