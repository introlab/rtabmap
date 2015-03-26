#import <CoreWLAN/CoreWLAN.h>
#include "WifiOSX.h"

int getRssi(const std::string& interfaceName)
{
    NSString* ifName = [NSString stringWithUTF8String:interfaceName.c_str()];
    CWInterface* interface = [CWInterface interfaceWithName:ifName];
    return interface.rssiValue;
}

std::vector<AccessPoint> scanAir(const std::string& interfaceName)
{
    NSString* ifName = [NSString stringWithUTF8String:interfaceName.c_str()];
    CWInterface* interface = [CWInterface interfaceWithName:ifName];
    
    NSError* error = nil;
    NSArray* scanResult = [[interface scanForNetworksWithSSID:nil error:&error] allObjects];
    if (error)
    {
        NSLog(@"%@ (%ld)", [error localizedDescription], [error code]);
    }
    
    std::vector<AccessPoint> result;
    for (CWNetwork* network in scanResult)
    {
        AccessPoint ap;
        ap.ssid  = std::string([[network ssid] UTF8String]);
        ap.bssid = std::string([[network bssid] UTF8String]);
        ap.rssi = [network rssiValue];
        result.push_back(ap);
    }
    
    return result;
}