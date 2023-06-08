#include <openvr_driver.h>
#include "driver_air_glasses.h"

#include <vector>
#include <thread>
#include <chrono>
#include <algorithm>
#include "../modules/xrealAirLinuxDriver/interface_lib/include/device3.h"
#include <math.h>
#include <string.h>

#if defined( _WINDOWS )
#include <windows.h>
#endif

#pragma warning (disable: 4996)

using namespace vr;


#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

inline HmdQuaternion_t HmdQuaternion_Init( double w, double x, double y, double z )
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

inline void HmdMatrix_SetIdentity( HmdMatrix34_t *pMatrix )
{
	pMatrix->m[0][0] = 1.f;
	pMatrix->m[0][1] = 0.f;
	pMatrix->m[0][2] = 0.f;
	pMatrix->m[0][3] = 0.f;
	pMatrix->m[1][0] = 0.f;
	pMatrix->m[1][1] = 1.f;
	pMatrix->m[1][2] = 0.f;
	pMatrix->m[1][3] = 0.f;
	pMatrix->m[2][0] = 0.f;
	pMatrix->m[2][1] = 0.f;
	pMatrix->m[2][2] = 1.f;
	pMatrix->m[2][3] = 0.f;
}


// keys for use with the settings API
static const char * const k_pch_steamvr_Section = "steamvr";
static const char * const k_pch_airGlasses_SerialNumber_String = "serialNumber";
static const char * const k_pch_airGlasses_ModelNumber_String = "modelNumber";
static const char * const k_pch_airGlasses_WindowX_Int32 = "windowX";
static const char * const k_pch_airGlasses_WindowY_Int32 = "windowY";
static const char * const k_pch_airGlasses_WindowWidth_Int32 = "windowWidth";
static const char * const k_pch_airGlasses_WindowHeight_Int32 = "windowHeight";
static const char * const k_pch_airGlasses_RenderWidth_Int32 = "renderWidth";
static const char * const k_pch_airGlasses_RenderHeight_Int32 = "renderHeight";
static const char * const k_pch_airGlasses_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char * const k_pch_airGlasses_DisplayFrequency_Float = "displayFrequency";

// own output settings
static const char * const k_pch_airGlasses_DistortionK1_Float = "DistortionK1";
static const char * const k_pch_airGlasses_DistortionK2_Float = "DistortionK2";
static const char * const k_pch_airGlasses_ZoomWidth_Float = "ZoomWidth";
static const char * const k_pch_airGlasses_ZoomHeight_Float = "ZoomHeight";
static const char * const k_pch_airGlasses_FOV_Float = "FOV";
static const char * const k_pch_airGlasses_DistanceBetweenEyes_Int32 = "DistanceBetweenEyes";
static const char * const k_pch_airGlasses_ScreenOffsetX_Int32 = "ScreenOffsetX";
static const char * const k_pch_airGlasses_Stereo_Bool = "Stereo";
static const char * const k_pch_airGlasses_DebugMode_Bool = "DebugMode";

// air glasses settings
static const char * const k_pch_airGlasses_Section = "air_glasses";
static const char * const k_pch_airGlasses_HMDRequire_Bool = "HMDRequire";
static const char * const k_pch_airGlasses_COM_port_Int32 = "COMPort";
static const char * const k_pch_airGlasses_CenteringKey_String = "CenteringKey";
static const char * const k_pch_airGlasses_CrouchPressKey_String = "CrouchPressKey";
static const char * const k_pch_airGlasses_CrouchOffset_Float = "CrouchOffset";

bool HMDConnected = false, HMDInitCentring = false, DeviceNotRequired = false;
float AirGlassesIMU[3] = { 0, 0, 0 }, yprOffset[3] = { 0, 0, 0 }; // Yaw, Pitch, Roll
float LastAirGlassesIMU[3] = { 0, 0, 0 };
double fPos[3] = { 0, 0, 0 };

#define StepPos 0.0033;
#define StepRot 0.2;

std::thread *pDeviceReadThread = NULL;

double DegToRad(double f) {
	return f * (3.14159265358979323846 / 180);
}

float OffsetYPR(float f, float f2)
{
	f -= f2;
	if (f < -180)
		f += 360;
	else if (f > 180)
		f -= 360;

	return f;
}

inline vr::HmdQuaternion_t EulerAngleToQuaternion(double Yaw, double Pitch, double Roll)
{
	vr::HmdQuaternion_t q;
	// Abbreviations for the various angular functions
	double cy = cos(Yaw * 0.5);
	double sy = sin(Yaw * 0.5);
	double cp = cos(Pitch * 0.5);
	double sp = sin(Pitch * 0.5);
	double cr = cos(Roll * 0.5);
	double sr = sin(Roll * 0.5);

	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

bool CorrectAngleValue(float Value)
{
	if (Value > -180 && Value < 180)
		return true;
	else
		return false;
}

void SetCentering()
{
	yprOffset[0] = AirGlassesIMU[0];
	yprOffset[1] = AirGlassesIMU[1];
	yprOffset[2] = AirGlassesIMU[2];
}

void AirGlassesIMURead(uint64_t timestamp,
                       device3_event_type event,
                       const device3_ahrs_type* ahrs) {
    static device3_quat_type old;
    static float dmax = -1.0f;

    if (event != DEVICE3_EVENT_UPDATE) {
        return;
    }

    device3_quat_type q = device3_get_orientation(ahrs);

    const float dx = (old.x - q.x) * (old.x - q.x);
    const float dy = (old.y - q.y) * (old.y - q.y);
    const float dz = (old.z - q.z) * (old.z - q.z);
    const float dw = (old.w - q.w) * (old.w - q.w);

    const float d = sqrtf(dx*dx + dy*dy + dz*dz + dw*dw);

    if (dmax < 0.0f) {
        dmax = 0.0f;
    } else {
        dmax = (d > dmax? d : dmax);
    }

    device3_vec3_type euler = device3_get_euler(q);
    AirGlassesIMU[0] = euler.x;
    AirGlassesIMU[1] = euler.y;
    AirGlassesIMU[2] = euler.z;

    if (CorrectAngleValue(AirGlassesIMU[0]) == false || CorrectAngleValue(AirGlassesIMU[1]) == false || CorrectAngleValue(AirGlassesIMU[2]) == false)
    {
        // last correct values
        AirGlassesIMU[0] = LastAirGlassesIMU[0];
        AirGlassesIMU[1] = LastAirGlassesIMU[1];
        AirGlassesIMU[2] = LastAirGlassesIMU[2];
    }
    else if (CorrectAngleValue(AirGlassesIMU[0]) && CorrectAngleValue(AirGlassesIMU[1]) && CorrectAngleValue(AirGlassesIMU[2])) // save last correct values
    {
        LastAirGlassesIMU[0] = AirGlassesIMU[0];
        LastAirGlassesIMU[1] = AirGlassesIMU[1];
        LastAirGlassesIMU[2] = AirGlassesIMU[2];

        if (HMDInitCentring == false)
            if (AirGlassesIMU[0] != 0 || AirGlassesIMU[1] != 0 || AirGlassesIMU[2] != 0) {
                SetCentering();
                HMDInitCentring = true;
            }
    }
}

void AirGlassesIMUThreadStart()
{
    device3_type* dev3 = device3_open(AirGlassesIMURead);
    device3_clear(dev3);

    while (dev3) {
        HMDConnected = true;
        if (device3_read(dev3, 0) < 0) {
            break;
        }
	}
	HMDConnected = false;

    device3_close(dev3);
}

void AirGlassesIMUStart() {
    pDeviceReadThread = new std::thread(AirGlassesIMUThreadStart);
}

//int KeyNameToKeyCode(std::string KeyName) {
//	std::transform(KeyName.begin(), KeyName.end(), KeyName.begin(), ::toupper);
//
//	if (KeyName == "NONE") return 0;
//
//	else if (KeyName == "MOUSE-LEFT-BTN") return VK_LBUTTON;
//	else if (KeyName == "MOUSE-RIGHT-BTN") return VK_RBUTTON;
//	else if (KeyName == "MOUSE-MIDDLE-BTN") return VK_MBUTTON;
//	else if (KeyName == "MOUSE-SIDE1-BTN") return VK_XBUTTON1;
//	else if (KeyName == "MOUSE-SIDE2-BTN") return VK_XBUTTON2;
//
//	else if (KeyName == "ESCAPE") return VK_ESCAPE;
//	else if (KeyName == "F1") return VK_F1;
//	else if (KeyName == "F2") return VK_F2;
//	else if (KeyName == "F3") return VK_F3;
//	else if (KeyName == "F4") return VK_F4;
//	else if (KeyName == "F5") return VK_F5;
//	else if (KeyName == "F6") return VK_F6;
//	else if (KeyName == "F7") return VK_F7;
//	else if (KeyName == "F8") return VK_F8;
//	else if (KeyName == "F9") return VK_F9;
//	else if (KeyName == "F10") return VK_F10;
//	else if (KeyName == "F11") return VK_F11;
//	else if (KeyName == "F12") return VK_F12;
//
//	else if (KeyName == "~") return 192;
//	else if (KeyName == "1") return '1';
//	else if (KeyName == "2") return '2';
//	else if (KeyName == "3") return '3';
//	else if (KeyName == "4") return '4';
//	else if (KeyName == "5") return '5';
//	else if (KeyName == "6") return '6';
//	else if (KeyName == "7") return '7';
//	else if (KeyName == "8") return '8';
//	else if (KeyName == "9") return '9';
//	else if (KeyName == "0") return '0';
//	else if (KeyName == "-") return 189;
//	else if (KeyName == "=") return 187;
//
//	else if (KeyName == "TAB") return VK_TAB;
//	else if (KeyName == "CAPS-LOCK") return VK_CAPITAL;
//	else if (KeyName == "SHIFT") return VK_SHIFT;
//	else if (KeyName == "CTRL") return VK_CONTROL;
//	else if (KeyName == "WIN") return VK_LWIN;
//	else if (KeyName == "ALT") return VK_MENU;
//	else if (KeyName == "SPACE") return VK_SPACE;
//	else if (KeyName == "ENTER") return VK_RETURN;
//	else if (KeyName == "BACKSPACE") return VK_BACK;
//
//	else if (KeyName == "Q") return 'Q';
//	else if (KeyName == "W") return 'W';
//	else if (KeyName == "E") return 'E';
//	else if (KeyName == "R") return 'R';
//	else if (KeyName == "T") return 'T';
//	else if (KeyName == "Y") return 'Y';
//	else if (KeyName == "U") return 'U';
//	else if (KeyName == "I") return 'I';
//	else if (KeyName == "O") return 'O';
//	else if (KeyName == "P") return 'P';
//	else if (KeyName == "[") return '[';
//	else if (KeyName == "]") return ']';
//	else if (KeyName == "A") return 'A';
//	else if (KeyName == "S") return 'S';
//	else if (KeyName == "D") return 'D';
//	else if (KeyName == "F") return 'F';
//	else if (KeyName == "G") return 'G';
//	else if (KeyName == "H") return 'H';
//	else if (KeyName == "J") return 'J';
//	else if (KeyName == "K") return 'K';
//	else if (KeyName == "L") return 'L';
//	else if (KeyName == ";") return 186;
//	else if (KeyName == "'") return 222;
//	else if (KeyName == "\\") return 220;
//	else if (KeyName == "Z") return 'Z';
//	else if (KeyName == "X") return 'X';
//	else if (KeyName == "C") return 'C';
//	else if (KeyName == "V") return 'V';
//	else if (KeyName == "B") return 'B';
//	else if (KeyName == "N") return 'N';
//	else if (KeyName == "M") return 'M';
//	else if (KeyName == "<") return 188;
//	else if (KeyName == ">") return 190;
//	else if (KeyName == "?") return 191;
//
//	else if (KeyName == "PRINTSCREEN") return VK_SNAPSHOT;
//	else if (KeyName == "SCROLL-LOCK") return VK_SCROLL;
//	else if (KeyName == "PAUSE") return VK_PAUSE;
//	else if (KeyName == "INSERT") return VK_INSERT;
//	else if (KeyName == "HOME") return VK_HOME;
//	else if (KeyName == "PAGE-UP") return VK_NEXT;
//	else if (KeyName == "DELETE") return VK_DELETE;
//	else if (KeyName == "END") return VK_END;
//	else if (KeyName == "PAGE-DOWN") return VK_PRIOR;
//
//	else if (KeyName == "UP") return VK_UP;
//	else if (KeyName == "DOWN") return VK_DOWN;
//	else if (KeyName == "LEFT") return VK_LEFT;
//	else if (KeyName == "RIGHT") return VK_RIGHT;
//
//	else if (KeyName == "NUM-LOCK") return VK_NUMLOCK;
//	else if (KeyName == "NUMPAD0") return VK_NUMPAD0;
//	else if (KeyName == "NUMPAD1") return VK_NUMPAD1;
//	else if (KeyName == "NUMPAD2") return VK_NUMPAD2;
//	else if (KeyName == "NUMPAD3") return VK_NUMPAD3;
//	else if (KeyName == "NUMPAD4") return VK_NUMPAD4;
//	else if (KeyName == "NUMPAD5") return VK_NUMPAD5;
//	else if (KeyName == "NUMPAD6") return VK_NUMPAD6;
//	else if (KeyName == "NUMPAD7") return VK_NUMPAD7;
//	else if (KeyName == "NUMPAD8") return VK_NUMPAD8;
//	else if (KeyName == "NUMPAD9") return VK_NUMPAD9;
//
//	else if (KeyName == "NUMPAD-DIVIDE") return VK_DIVIDE;
//	else if (KeyName == "NUMPAD-MULTIPLY") return VK_MULTIPLY;
//	else if (KeyName == "NUMPAD-MINUS") return VK_SUBTRACT;
//	else if (KeyName == "NUMPAD-PLUS") return VK_ADD;
//	else if (KeyName == "NUMPAD-DEL") return VK_DECIMAL;
//
//	else return 0;
//}


CDeviceDriver::CDeviceDriver(  )
{
    vr::VRDriverLog()->Log("air_glasses driver Init");
    m_hmdId = vr::k_unTrackedDeviceIndexInvalid;
    m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

    m_flIPD = vr::VRSettings()->GetFloat(k_pch_steamvr_Section, k_pch_SteamVR_IPD_Float );

    char buf[1024];
    vr::VRSettings()->GetString(k_pch_steamvr_Section, k_pch_airGlasses_SerialNumber_String, buf, sizeof( buf ) );
    m_sSerialNumber = buf;

    vr::VRSettings()->GetString(k_pch_steamvr_Section, k_pch_airGlasses_ModelNumber_String, buf, sizeof( buf ) );
    m_sModelNumber = buf;

    m_nWindowX = vr::VRSettings()->GetInt32(k_pch_steamvr_Section, k_pch_airGlasses_WindowX_Int32 );
    m_nWindowY = vr::VRSettings()->GetInt32(k_pch_steamvr_Section, k_pch_airGlasses_WindowY_Int32 );
    m_nWindowWidth = vr::VRSettings()->GetInt32(k_pch_steamvr_Section, k_pch_airGlasses_WindowWidth_Int32 );
    m_nWindowHeight = vr::VRSettings()->GetInt32(k_pch_steamvr_Section, k_pch_airGlasses_WindowHeight_Int32 );
    m_nRenderWidth = vr::VRSettings()->GetInt32(k_pch_steamvr_Section, k_pch_airGlasses_RenderWidth_Int32 );
    m_nRenderHeight = vr::VRSettings()->GetInt32(k_pch_steamvr_Section, k_pch_airGlasses_RenderHeight_Int32 );
    m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(k_pch_steamvr_Section, k_pch_airGlasses_SecondsFromVsyncToPhotons_Float );
    m_flDisplayFrequency = vr::VRSettings()->GetFloat(k_pch_steamvr_Section, k_pch_airGlasses_DisplayFrequency_Float );

    m_fDistortionK1 = vr::VRSettings()->GetFloat(k_pch_steamvr_Section, k_pch_airGlasses_DistortionK1_Float);
    m_fDistortionK2 = vr::VRSettings()->GetFloat(k_pch_steamvr_Section, k_pch_airGlasses_DistortionK2_Float);
    m_fZoomWidth = vr::VRSettings()->GetFloat(k_pch_steamvr_Section, k_pch_airGlasses_ZoomWidth_Float);
    m_fZoomHeight = vr::VRSettings()->GetFloat(k_pch_steamvr_Section, k_pch_airGlasses_ZoomHeight_Float);
    m_fFOV = (vr::VRSettings()->GetFloat(k_pch_steamvr_Section, k_pch_airGlasses_FOV_Float) * 3.14159265358979323846 / 180); //radians
    m_nDistanceBetweenEyes = vr::VRSettings()->GetInt32(k_pch_steamvr_Section, k_pch_airGlasses_DistanceBetweenEyes_Int32);
    m_nScreenOffsetX = vr::VRSettings()->GetInt32(k_pch_steamvr_Section, k_pch_airGlasses_ScreenOffsetX_Int32);
    m_bStereoMode = vr::VRSettings()->GetBool(k_pch_steamvr_Section, k_pch_airGlasses_Stereo_Bool);
    m_bDebugMode = vr::VRSettings()->GetBool(k_pch_steamvr_Section, k_pch_airGlasses_DebugMode_Bool);

//		vr::VRSettings()->GetString(k_pch_airGlasses_Section, k_pch_airGlasses_CenteringKey_String, buf, sizeof(buf));
//		m_centeringKey = KeyNameToKeyCode(buf);
//
//		vr::VRSettings()->GetString(k_pch_airGlasses_Section, k_pch_airGlasses_CrouchPressKey_String, buf, sizeof(buf));
//		m_crouchPressKey = KeyNameToKeyCode(buf);
//
//		m_crouchOffset = vr::VRSettings()->GetFloat(k_pch_airGlasses_Section, k_pch_airGlasses_CrouchOffset_Float);
//
//		vr::VRSettings()->GetString(k_pch_airGlasses_Section, "HMDUpKey", buf, sizeof(buf));
//		m_hmdUpKey = KeyNameToKeyCode(buf);
//
//		vr::VRSettings()->GetString(k_pch_airGlasses_Section, "HMDDownKey", buf, sizeof(buf));
//		m_hmdDownKey = KeyNameToKeyCode(buf);
//
//		vr::VRSettings()->GetString(k_pch_airGlasses_Section, "HMDResetKey", buf, sizeof(buf));
//		m_hmdResetKey = KeyNameToKeyCode(buf);

    vr::VRDriverLog()->Log("driver_air_glasses: Serial Number: ");
    vr::VRDriverLog()->Log(m_sSerialNumber.c_str());
    vr::VRDriverLog()->Log("driver_air_glasses: Model Number: ");
    vr::VRDriverLog()->Log(m_sModelNumber.c_str());
//    vr::VRDriverLog()->Log( "driver_air_glasses: Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
//    vr::VRDriverLog()->Log( "driver_air_glasses: Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
//    vr::VRDriverLog()->Log( "driver_air_glasses: Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
//    vr::VRDriverLog()->Log( "driver_air_glasses: Display Frequency: %f\n", m_flDisplayFrequency );
//    vr::VRDriverLog()->Log( "driver_air_glasses: IPD: %f\n", m_flIPD );
}

vr::EVRInitError CDeviceDriver::Activate( uint32_t unObjectId )
{
    vr::VRDriverLog()->Log("air_glasses driver Activate");
    m_hmdId = unObjectId;
    m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_hmdId);

    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str() );
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str() );
    vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD );
    vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f );
    vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency );
    vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons );

    // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
    vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

    // avoid "not fullscreen" warnings from vrmonitor
    vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false );

    // debug mode activate Windowed Mode (borderless fullscreen), lock to 30 FPS
    vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_DisplayDebugMode_Bool, m_bDebugMode );

    // Icons can be configured in code or automatically configured by an external file "drivername\resources\driver.vrresources".
    // Icon properties NOT configured in code (post Activate) are then auto-configured by the optional presence of a driver's "drivername\resources\driver.vrresources".
    // In this manner a driver can configure their icons in a flexible data driven fashion by using an external file.
    //
    // The structure of the driver.vrresources file allows a driver to specialize their icons based on their HW.
    // Keys matching the value in "Prop_ModelNumber_String" are considered first, since the driver may have model specific icons.
    // An absence of a matching "Prop_ModelNumber_String" then considers the ETrackedDeviceClass ("HMD", "Controller", "GenericTracker", "TrackingReference")
    // since the driver may have specialized icons based on those device class names.
    //
    // An absence of either then falls back to the "system.vrresources" where generic device class icons are then supplied.
    //
    // Please refer to "bin\drivers\sample\resources\driver.vrresources" which contains this sample configuration.
    //
    // "Alias" is a reserved key and specifies chaining to another json block.
    //
    // In this sample configuration file (overly complex FOR EXAMPLE PURPOSES ONLY)....
    //
    // "Model-v2.0" chains through the alias to "Model-v1.0" which chains through the alias to "Model-v Defaults".
    //
    // Keys NOT found in "Model-v2.0" would then chase through the "Alias" to be resolved in "Model-v1.0" and either resolve their or continue through the alias.
    // Thus "Prop_NamedIconPathDeviceAlertLow_String" in each model's block represent a specialization specific for that "model".
    // Keys in "Model-v Defaults" are an example of mapping to the same states, and here all map to "Prop_NamedIconPathDeviceOff_String".
    //
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{indexhmd}/icons/headset_status_off.png" );
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{indexhmd}/icons/headset_status_searching.gif" );
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{indexhmd}/icons/headset_status_searching_alert.gif" );
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{indexhmd}/icons/headset_status_ready.png" );
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{indexhmd}/icons/headset_status_ready_alert.png" );
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{indexhmd}/icons/headset_status_error.png" );
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{indexhmd}/icons/headset_status_standby.png" );
    vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{indexhmd}/icons/headset_status_standby.png" ); //headset_status_ready_low.png

    return VRInitError_None;
}

void CDeviceDriver::Deactivate()
{
    m_hmdId = vr::k_unTrackedDeviceIndexInvalid;
}

void CDeviceDriver::EnterStandby()
{
}

void *CDeviceDriver::GetComponent( const char *pchComponentNameAndVersion )
{
    if ( !strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) )
    {
        return (vr::IVRDisplayComponent*)this;
    }

    // override this to add a component to a driver
    return NULL;
}

/** debug request from a client */
void CDeviceDriver::DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
{
    if( unResponseBufferSize >= 1 )
        pchResponseBuffer[0] = 0;
}

bool CDeviceDriver::IsDisplayOnDesktop()
{
    return true;
}

bool CDeviceDriver::IsDisplayRealDisplay()
{
    return false; //Support working on extended display
}

void CDeviceDriver::GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
{
    *pnWidth = m_nRenderWidth;
    *pnHeight = m_nRenderHeight;
}

//-----------------------------------------------------------------------------
// Purpose: To inform vrcompositor how the screens should be organized.
//-----------------------------------------------------------------------------
void CDeviceDriver::GetEyeOutputViewport( vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnY = 0;

	// Each eye will have half the window
	*pnWidth = m_nWindowWidth / 2;

	// Each eye will have the full height
	*pnHeight = m_nWindowHeight;

	if ( eEye == vr::Eye_Left )
	{
		// Left eye viewport on the left half of the window
		*pnX = 0;
	}
	else
	{
		// Right eye viewport on the right half of the window
		*pnX = m_nWindowWidth / 2;
	}
}

//-----------------------------------------------------------------------------
// Purpose: To inform the compositor what the projection parameters are for this HMD.
//-----------------------------------------------------------------------------
void CDeviceDriver::GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom )
{
	*pfLeft = -1.0;
	*pfRight = 1.0;
	*pfTop = -1.0;
	*pfBottom = 1.0;
}

//-----------------------------------------------------------------------------
// Purpose: To compute the distortion properties for a given uv in an image.
//-----------------------------------------------------------------------------
vr::DistortionCoordinates_t CDeviceDriver::ComputeDistortion( vr::EVREye eEye, float fU, float fV )
{
	vr::DistortionCoordinates_t coordinates{};
	coordinates.rfBlue[ 0 ] = fU;
	coordinates.rfBlue[ 1 ] = fV;
	coordinates.rfGreen[ 0 ] = fU;
	coordinates.rfGreen[ 1 ] = fV;
	coordinates.rfRed[ 0 ] = fU;
	coordinates.rfRed[ 1 ] = fV;
	return coordinates;
}

//-----------------------------------------------------------------------------
// Purpose: To inform vrcompositor what the window bounds for this virtual HMD are.
//-----------------------------------------------------------------------------
void CDeviceDriver::GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
    *pnX = m_nWindowX;
    *pnY = m_nWindowY;
    *pnWidth = m_nWindowWidth;
    *pnHeight = m_nWindowHeight;
}

DriverPose_t CDeviceDriver::GetPose()
{
    DriverPose_t pose = { 0 };

    if (HMDConnected || DeviceNotRequired) {
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
    } else {
        pose.poseIsValid = false;
        pose.result = TrackingResult_Uninitialized;
        pose.deviceIsConnected = false;
    }

    pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
    pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

    if (HMDConnected || DeviceNotRequired) {
        // Set head tracking rotation
        pose.qRotation = EulerAngleToQuaternion(DegToRad( OffsetYPR(AirGlassesIMU[0], yprOffset[0]) * -1  ),
                                                DegToRad( OffsetYPR(AirGlassesIMU[2], yprOffset[2])),
                                                DegToRad( OffsetYPR(AirGlassesIMU[1], yprOffset[1]) ));
    }

    return pose;
}

void CDeviceDriver::RunFrame()
{
    if (m_hmdId != vr::k_unTrackedDeviceIndexInvalid )
    {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated( m_hmdId, GetPose(), sizeof( DriverPose_t ) );
    }
}

std::string CDeviceDriver::GetSerialNumber()
{
    return m_sSerialNumber;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver: public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
	virtual void Cleanup() ;
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame() ;
	virtual bool ShouldBlockStandbyMode()  { return false; }
	virtual void EnterStandby()  {}
	virtual void LeaveStandby()  {}


private:
	CDeviceDriver *m_pNullHmdLatest = nullptr;
};

CServerDriver g_serverDriverNull;


EVRInitError CServerDriver::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );

	DeviceNotRequired = !vr::VRSettings()->GetInt32(k_pch_airGlasses_Section, k_pch_airGlasses_HMDRequire_Bool);
	if (HMDConnected || DeviceNotRequired)
	{
		m_pNullHmdLatest = new CDeviceDriver();
		if (!vr::VRServerDriverHost()->TrackedDeviceAdded(m_pNullHmdLatest->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_pNullHmdLatest))
		{
		    vr::VRDriverLog()->Log( "Failed to create hmd device!" );
		    return vr::VRInitError_Driver_Unknown;
		}
	}

    vr::VRDriverLog()->Log("Device driver created!");

	AirGlassesIMUStart();

	return VRInitError_None;
}

void CServerDriver::Cleanup()
{
	if (HMDConnected) {
		HMDConnected = false;
		pDeviceReadThread->join();
		delete pDeviceReadThread;
		pDeviceReadThread = nullptr;
	}

	if (HMDConnected || DeviceNotRequired) {
		delete m_pNullHmdLatest;
		m_pNullHmdLatest = NULL;
	}
}


void CServerDriver::RunFrame()
{
	if ( m_pNullHmdLatest )
	{
		m_pNullHmdLatest->RunFrame();
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode )
{
	if( 0 == strcmp( IServerTrackedDeviceProvider_Version, pInterfaceName ) )
	{
		return &g_serverDriverNull;
	}

	if( pReturnCode )
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}
