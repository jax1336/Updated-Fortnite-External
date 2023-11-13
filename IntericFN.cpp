#include "Electric.h"
#include "driver.h"
#include "Print.hpp"
#include "d3d9_x.h"
#include "xor.hpp"
#include <dwmapi.h>
#include "Dog.h"
#include "lazy.h"
#include <Windows.h>
#include "auth.hpp"
#include <string>
#include <vector>
#include "Animal.h"
#include "Keybind.h"
#include "offsets.h"
#include "bytes.h"
#include "models.h"
#include "icons1.h"
#include <d3d9.h>
//#include "sk.hpp"

// dont sell this shit or you get exposed hard


DWORD FindProcessID(std::string name) {
	PROCESSENTRY32 ProcessEntry;
	ProcessEntry.dwSize = sizeof(PROCESSENTRY32);
	HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, NULL);

	if (Process32First(snapshot, &ProcessEntry) == (int)1) {
		while (Process32Next(snapshot, &ProcessEntry) == (int)1) {
			if (std::string(ProcessEntry.szExeFile) == name)
				return ProcessEntry.th32ProcessID;
		}
	}

	CloseHandle(snapshot);
	return NULL;
}


#define color1 (WORD)(0x0001 | 0x0000)
#define color2 (WORD)(0x0002 | 0x0000)
#define color3 (WORD)(0x0003 | 0x0000)
#define color4 (WORD)(0x0004 | 0x0000)
#define color5 (WORD)(0x0005 | 0x0000)
#define color6 (WORD)(0x0006 | 0x0000)
#define color7 (WORD)(0x0007 | 0x0000)
#define color8 (WORD)(0x0008 | 0x0000)
#define color9 (WORD)(0x0008 | 0x0000)
#define COLOR(h, c) SetConsoleTextAttribute(h, c);

static int aimkeypos = 3;
static int aimbone = 1;

int faken_rot = 0;

float BOG_TO_GRD(float BOG) {
	return (180 / M_PI) * BOG;
}

float GRD_TO_BOG(float GRD) {
	return (M_PI / 180) * GRD;
}

bool lobby = false;
bool aimlineesp;
bool fuel = false;
//bool TopCorner;
bool exploitss = false;
bool particles = false;
bool VisibleCheck = true;
bool SKELETONesp;
bool animate = false;

bool nazi2 = false;
bool threed = true;
bool filledsqr = true;
bool trigbot;
bool triggerbot;
bool actorskeleton;

float color_red = 1.;
float color_green = 0;
float color_blue = 0;


bool fovcircle = false;
bool targetlines = true;
bool ShowMenu = true;
bool rainbowesp;
bool Esp = true;
bool Esp_box = true;
bool cornerbox = false;
bool Esp_info = true;
bool Esp_line = false;
bool Aimbot = true;
bool Skeleton = false;
bool slefESP = false;
bool square_fov = false;
bool weaponesp = false;
bool BOXESP = true;
bool ammoESP = true;
bool AimWhileJumping = false;
bool Esp_Distance = true;
bool carFly = false;
bool niggerfovchanger = false;
bool RapidFire = false;
bool spinbot = false;
bool boatspeed = false;
bool bostspeed = false;
bool carto = false;
bool Safemode = false;
bool fillbox = true;
bool Fillbox = true;
bool fovcirclefilled = false;
bool lineheadesp = false;
bool cornerline = false;
bool watermark = true;
bool fpsCounter = true;
bool reloadCheck = true;
bool WindowStreamProof = false;
bool HeadESP = false;
bool crosshair = false;

float BoxWidthValue = 0.350;



float ChangerFOV = 80;

__int64 TargetedBuild;
bool bTargetedBuild;


ImFont* m_pFont;
float smooth = 5;
static int VisDist = 100;
float AimFOV = 150;
static int aimkey;
static int hitbox;
static int hitboxpos = 0;


float CrossHair[3];
float CrossThickness = 0.8f;

Memory* KmDrv = nullptr;

DWORD_PTR Uworld;
DWORD_PTR LocalPawn;
DWORD_PTR PlayerState;
DWORD_PTR Localplayer;
DWORD_PTR Rootcomp;
DWORD_PTR PlayerController;
DWORD_PTR Persistentlevel;
uintptr_t PlayerCameraManager;
Vector3 localactorpos;

IDirect3DTexture9* logo = nullptr;
IDirect3DTexture9* option1 = nullptr;
IDirect3DTexture9* option2 = nullptr;
IDirect3DTexture9* option3 = nullptr;

//Helpers
ImFont* details;
ImFont* arrow;
ImFont* mainfont;
ImFont* watermarkfont;
ImFont* titlefont;
ImFont* icons;
ImVec2 position;
ImDrawList* drawlist;
ImVec2 watermarkposition;
ImDrawList* watermarkdrawlist;
ImVec2 skeletonpos;
ImDrawList* skeletondraw;
int tabs = 0;

uint64_t TargetPawn;
int localplayerID;

RECT GameRect = { NULL };
D3DPRESENT_PARAMETERS d3dpp;
DWORD ScreenCenterX;
DWORD ScreenCenterY;
Vector3 LocalRelativeLocation; struct FBoxSphereBounds
{
	struct Vector3                                     Origin;                                                   // 0x0000(0x0018) (Edit, BlueprintVisible, ZeroConstructor, SaveGame, IsPlainOldData)
	struct Vector3                                     BoxExtent;                                                // 0x0018(0x0018) (Edit, BlueprintVisible, ZeroConstructor, SaveGame, IsPlainOldData)
	double                                             SphereRadius;                                             // 0x0030(0x0008) (Edit, BlueprintVisible, ZeroConstructor, SaveGame, IsPlainOldData)
};
static void xCreateWindow();
static void xInitD3d();
static void xMainLoop();
static void xShutdown();
static LRESULT CALLBACK WinProc(HWND hWnd, UINT Message, WPARAM wParam, LPARAM lParam);
extern LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static HWND Window = NULL;
IDirect3D9Ex* p_Object = NULL;
static LPDIRECT3DDEVICE9 D3dDevice = NULL;
static LPDIRECT3DVERTEXBUFFER9 TriBuf = NULL;

typedef struct {
	float X, Y, Z;
} FVector;

typedef struct {
	float X, Y;
} FVector2D;



typedef struct {
	FVector Location;
	float FOV;
	float OrthoWidth;
	float OrthoNearClipPlane;
	float OrthoFarClipPlane;
	float AspectRatio;
} FMinimalViewInfo;

typedef struct {
	float M[4][4];
} FMatrix;

typedef struct {
	FVector ViewOrigin;
	char _padding_0[4];
	FMatrix ViewRotationMatrix;
	FMatrix ProjectionMatrix;
} FSceneViewProjectionData;




class UClass {
public:
	BYTE _padding_0[0x40];
	UClass* SuperClass;
};

class UObject {
public:
	PVOID VTableObject;
	DWORD ObjectFlags;
	DWORD InternalIndex;
	UClass* Class;
	BYTE _padding_0[0x8];
	UObject* Outer;

	inline BOOLEAN IsA(PVOID parentClass) {
		for (auto super = this->Class; super; super = super->SuperClass) {
			if (super == parentClass) {
				return TRUE;
			}
		}

		return FALSE;
	}
};

void DrawLine(int x1, int y1, int x2, int y2, RGBA* color, int thickness)
{
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 255.0, color->B / 255.0, color->A / 255.0)), thickness);
}


class FUObjectItem {
public:
	UObject* Object;
	DWORD Flags;
	DWORD ClusterIndex;
	DWORD SerialNumber;
	DWORD SerialNumber2;
};

class TUObjectArray {
public:
	FUObjectItem* Objects[9];
};

class GObjects {
public:
	TUObjectArray* ObjectArray;
	BYTE _padding_0[0xC];
	DWORD ObjectCount;
};


template<class T>
struct TArray {
	friend struct FString;

public:
	inline TArray() {
		Data = nullptr;
		Count = Max = 0;
	};

	inline INT Num() const {
		return Count;
	};

	inline T& operator[](INT i) {
		return Data[i];
	};

	inline BOOLEAN IsValidIndex(INT i) {
		return i < Num();
	}

private:
	T* Data;
	INT Count;
	INT Max;
};

struct FString : private TArray<WCHAR> {
	FString() {
		Data = nullptr;
		Max = Count = 0;
	}

	FString(LPCWSTR other) {
		Max = Count = static_cast<INT>(wcslen(other));

		if (Count) {
			Data = const_cast<PWCHAR>(other);
		}
	};

	inline BOOLEAN IsValid() {
		return Data != nullptr;
	}

	inline PWCHAR c_str() {
		return Data;
	}
};

VOID(*FreeInternal)(PVOID) = nullptr;

VOID Free(PVOID buffer) {
	FreeInternal(buffer);
}


#pragma once

namespace detail
{
	extern "C" void* _spoofer_stub();

	template <typename Ret, typename... Args>
	static inline auto shellcode_stub_helper(
		const void* shell,
		Args... args
	) -> Ret
	{
		auto fn = (Ret(*)(Args...))(shell);
		return fn(args...);
	}

	template <std::size_t Argc, typename>
	struct argument_remapper
	{
		template<
			typename Ret,
			typename First,
			typename Second,
			typename Third,
			typename Fourth,
			typename... Pack
		>
		static auto do_call(const void* shell, void* shell_param, First first, Second second,
			Third third, Fourth fourth, Pack... pack) -> Ret
		{
			return shellcode_stub_helper< Ret, First, Second, Third, Fourth, void*, void*, Pack... >(shell, first, second, third, fourth, shell_param, nullptr, pack...);
		}
	};

	template <std::size_t Argc>
	struct argument_remapper<Argc, std::enable_if_t<Argc <= 4>>
	{
		template<
			typename Ret,
			typename First = void*,
			typename Second = void*,
			typename Third = void*,
			typename Fourth = void*
		>
		static auto do_call(
			const void* shell,
			void* shell_param,
			First first = First{},
			Second second = Second{},
			Third third = Third{},
			Fourth fourth = Fourth{}
		) -> Ret
		{
			return shellcode_stub_helper<
				Ret,
				First,
				Second,
				Third,
				Fourth,
				void*,
				void*
			>(
				shell,
				first,
				second,
				third,
				fourth,
				shell_param,
				nullptr
				);
		}
	};
}

template <typename Ret, typename... Args>
static inline auto SpoofCall(Ret(*fn)(Args...), Args... args) -> Ret
{
	static const void* jmprbx = nullptr;
	if (!jmprbx) {
		const auto ntdll = reinterpret_cast<const unsigned char*>(::GetModuleHandleW(NULL));
		const auto dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(ntdll);
		const auto nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(ntdll + dos->e_lfanew);
		const auto sections = IMAGE_FIRST_SECTION(nt);
		const auto num_sections = nt->FileHeader.NumberOfSections;

		constexpr char section_name[5]{ '.', 't', 'e', 'x', 't' };
		const auto     section = std::find_if(sections, sections + num_sections, [&](const auto& s) {
			return std::equal(s.Name, s.Name + 5, section_name);
			});

		constexpr unsigned char instr_bytes[2]{ 0xFF, 0x26 };
		const auto              va = ntdll + section->VirtualAddress;
		jmprbx = std::search(va, va + section->Misc.VirtualSize, instr_bytes, instr_bytes + 2);
	}

	struct shell_params
	{
		const void* trampoline;
		void* function;
		void* rdx;
	};

	shell_params p
	{
		jmprbx,
		reinterpret_cast<void*>(fn)
	};

	using mapper = detail::argument_remapper<sizeof...(Args), void>;
	return mapper::template do_call<Ret, Args...>((const void*)&detail::_spoofer_stub, &p, args...);
}

namespace SpoofRuntime {
	inline float acosf_(float x)
	{
		return SpoofCall(acosf, x);
	}

	inline float atan2f_(float x, float y)
	{
		return SpoofCall(atan2f, x, y);
	}
}
std::wstring MBytesToWString(const char* lpcszString)
{
	int len = strlen(lpcszString);
	int unicodeLen = ::MultiByteToWideChar(CP_ACP, 0, lpcszString, -1, NULL, 0);
	wchar_t* pUnicode = new wchar_t[unicodeLen + 1];
	memset(pUnicode, 0, (unicodeLen + 1) * sizeof(wchar_t));
	::MultiByteToWideChar(CP_ACP, 0, lpcszString, -1, (LPWSTR)pUnicode, unicodeLen);
	std::wstring wString = (wchar_t*)pUnicode;
	delete[] pUnicode;
	return wString;
}
std::string WStringToUTF8(const wchar_t* lpwcszWString)
{
	char* pElementText;
	int iTextLen = ::WideCharToMultiByte(CP_UTF8, 0, (LPWSTR)lpwcszWString, -1, NULL, 0, NULL, NULL);
	pElementText = new char[iTextLen + 1];
	memset((void*)pElementText, 0, (iTextLen + 1) * sizeof(char));
	::WideCharToMultiByte(CP_UTF8, 0, (LPWSTR)lpwcszWString, -1, pElementText, iTextLen, NULL, NULL);
	std::string strReturn(pElementText);
	delete[] pElementText;
	return strReturn;
}
void DrawString(float fontSize, int x, int y, RGBA* color, bool bCenter, bool stroke, const char* pText, ...)
{
	va_list va_alist;
	char buf[1024] = { 0 };
	va_start(va_alist, pText);
	_vsnprintf_s(buf, sizeof(buf), pText, va_alist);
	va_end(va_alist);
	std::string text = WStringToUTF8(MBytesToWString(buf).c_str());
	if (bCenter)
	{
		ImVec2 textSize = ImGui::CalcTextSize(text.c_str());
		x = x - textSize.x / 4;
		y = y - textSize.y;
	}
	if (stroke)
	{
		ImGui::GetOverlayDrawList()->AddText(ImGui::GetFont(), fontSize, ImVec2(x + 1, y + 1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), text.c_str());
		ImGui::GetOverlayDrawList()->AddText(ImGui::GetFont(), fontSize, ImVec2(x - 1, y - 1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), text.c_str());
		ImGui::GetOverlayDrawList()->AddText(ImGui::GetFont(), fontSize, ImVec2(x + 1, y - 1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), text.c_str());
		ImGui::GetOverlayDrawList()->AddText(ImGui::GetFont(), fontSize, ImVec2(x - 1, y + 1), ImGui::ColorConvertFloat4ToU32(ImVec4(0, 0, 0, 1)), text.c_str());
	}
	ImGui::GetOverlayDrawList()->AddText(ImGui::GetFont(), fontSize, ImVec2(x, y), ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 153.0, color->B / 51.0, color->A / 255.0)), text.c_str());
}

#define BONE_HEAD_ID (68)
#define BONE_NECK_ID (67)
#define BONE_CHEST_ID (36)
#define BONE_PELVIS_ID (2)




Vector3 GetBoneWithRotation(uintptr_t mesh, int bone_id)
{
	uintptr_t bone_array = KmDrv->Rpm<uintptr_t>(mesh + OFFSETS::BoneArray);
	if (bone_array == NULL) bone_array = KmDrv->Rpm<uintptr_t>(mesh + OFFSETS::BoneArray + 0x10);
	FTransform bone = KmDrv->Rpm<FTransform>(bone_array + (bone_id * 0x60));
	FTransform component_to_world = KmDrv->Rpm<FTransform>(mesh + OFFSETS::ComponetToWorld);
	D3DMATRIX matrix = MatrixMultiplication(bone.ToMatrixWithScale(), component_to_world.ToMatrixWithScale());
	return Vector3(matrix._41, matrix._42, matrix._43);
}

D3DXMATRIX Matrix(Vector3 rot, Vector3 origin = Vector3(0, 0, 0))
{
	float radPitch = (rot.x * float(M_PI) / 180.f);
	float radYaw = (rot.y * float(M_PI) / 180.f);
	float radRoll = (rot.z * float(M_PI) / 180.f);

	float SP = sinf(radPitch);
	float CP = cosf(radPitch);
	float SY = sinf(radYaw);
	float CY = cosf(radYaw);
	float SR = sinf(radRoll);
	float CR = cosf(radRoll);

	D3DMATRIX matrix;
	matrix.m[0][0] = CP * CY;
	matrix.m[0][1] = CP * SY;
	matrix.m[0][2] = SP;
	matrix.m[0][3] = 0.f;

	matrix.m[1][0] = SR * SP * CY - CR * SY;
	matrix.m[1][1] = SR * SP * SY + CR * CY;
	matrix.m[1][2] = -SR * CP;
	matrix.m[1][3] = 0.f;

	matrix.m[2][0] = -(CR * SP * CY + SR * SY);
	matrix.m[2][1] = CY * SR - CR * SP * SY;
	matrix.m[2][2] = CR * CP;
	matrix.m[2][3] = 0.f;

	matrix.m[3][0] = origin.x;
	matrix.m[3][1] = origin.y;
	matrix.m[3][2] = origin.z;
	matrix.m[3][3] = 1.f;

	return matrix;
}

//auto hijack() -> bool {
//	Window = FindWindowA(E("MedalOverlayClass"), E("MedalOverlay"));
//	if (!Window) return false;
//	printf("hijacked medal cuzzzzzz \n");
//	MARGINS Margin = { -1 };
//	DwmExtendFrameIntoClientArea(Window, &Margin);
//	SetWindowPos(Window, 0, 0, 0, GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN), SWP_SHOWWINDOW);
//	ShowWindow(Window, SW_SHOW);
//	UpdateWindow(Window);
//	return true;
//
//}



struct CamewaDescwipsion
{
	Vector3 location;
	Vector3 rotation;
	float fov;
};

CamewaDescwipsion get_camera()
{
	CamewaDescwipsion camera;

	auto location_pointer = KmDrv->Rpm<uintptr_t>(Uworld + 0x110);
	auto rotation_pointer = KmDrv->Rpm<uintptr_t>(Uworld + 0x120);

	struct FNRot
	{
		double a; //0x0000
		char pad_0008[24]; //0x0008
		double b; //0x0020
		char pad_0028[424]; //0x0028
		double c; //0x01D0
	}fnRot;

	fnRot.a = KmDrv->Rpm<double>(rotation_pointer);
	fnRot.b = KmDrv->Rpm<double>(rotation_pointer + 0x20);
	fnRot.c = KmDrv->Rpm<double>(rotation_pointer + 0x1d0);

	camera.location = KmDrv->Rpm<Vector3>(location_pointer);
	camera.rotation.x = asin(fnRot.c) * (180.0 / M_PI);
	camera.rotation.y = ((atan2(fnRot.a * -1, fnRot.b) * (180.0 / M_PI)) * -1) * -1;
	camera.fov = KmDrv->Rpm<float>((uintptr_t)PlayerController + 0x394) * 90.f;

	return camera;
}

float fov;

Vector3 ProjectWorldToScreen(Vector3 WorldLocation)
{
	CamewaDescwipsion ViewPoint = get_camera();
	D3DMATRIX tempMatrix = Matrix(ViewPoint.rotation);
	Vector3 vAxisX = Vector3(tempMatrix.m[0][0], tempMatrix.m[0][1], tempMatrix.m[0][2]);
	Vector3 vAxisY = Vector3(tempMatrix.m[1][0], tempMatrix.m[1][1], tempMatrix.m[1][2]);
	Vector3 vAxisZ = Vector3(tempMatrix.m[2][0], tempMatrix.m[2][1], tempMatrix.m[2][2]);
	Vector3 vDelta = WorldLocation - ViewPoint.location;
	Vector3 vTransformed = Vector3(vDelta.Dot(vAxisY), vDelta.Dot(vAxisZ), vDelta.Dot(vAxisX));
	if (vTransformed.z < 1.f)
		vTransformed.z = 1.f;
	return Vector3((Width / 2.0f) + vTransformed.x * (((Width / 2.0f) / tanf(ViewPoint.fov * (float)M_PI / 360.f))) / vTransformed.z, (Height / 2.0f) - vTransformed.y * (((Width / 2.0f) / tanf(ViewPoint.fov * (float)M_PI / 360.f))) / vTransformed.z, 0);
}


//dont sell this its github paste you faggot now kys
void DrawStrokeText(int x, int y, const char* str)
{
	ImFont a;
	std::string utf_8_1 = std::string(str);
	std::string utf_8_2 = string_To_UTF8(utf_8_1);

	ImGui::GetOverlayDrawList()->AddText(ImVec2(x, y - 1), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), utf_8_2.c_str());
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x, y + 1), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), utf_8_2.c_str());
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x - 1, y), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), utf_8_2.c_str());
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x + 1, y), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), utf_8_2.c_str());
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x, y), ImGui::ColorConvertFloat4ToU32(ImVec4(255, 255, 255, 255)), utf_8_2.c_str());
}



void skeleton(uint64_t acotrmesh, bool isvisible)
{
	//Vector3 neck2 = GetBoneWithRotation(acotrmesh, 68);
	//Vector3 vneck2 = ProjectWorldToScreen(neck2);


	//Vector3 neck = GetBoneWithRotation(acotrmesh, 66);
	//Vector3 vneck = ProjectWorldToScreen(neck);
	//Vector3 rightChest = GetBoneWithRotation(acotrmesh, 8);
	//Vector3 vrightChest = ProjectWorldToScreen(rightChest);
	//Vector3 leftChest = GetBoneWithRotation(acotrmesh, 37);
	//Vector3 vleftChest = ProjectWorldToScreen(leftChest);
	//Vector3 leftShoulder = GetBoneWithRotation(acotrmesh, 38);
	//Vector3 vleftShoulder = ProjectWorldToScreen(leftShoulder);
	//Vector3 rightShoulder = GetBoneWithRotation(acotrmesh, 9);
	//Vector3 vrightShoulder = ProjectWorldToScreen(rightShoulder);
	//Vector3 leftElbow = GetBoneWithRotation(acotrmesh, 39);
	//Vector3 vleftElbow = ProjectWorldToScreen(leftElbow);
	//Vector3 rightElbow = GetBoneWithRotation(acotrmesh, 10);
	//Vector3 vrightElbow = ProjectWorldToScreen(rightElbow);
	//Vector3 leftWrist = GetBoneWithRotation(acotrmesh, 62);
	//Vector3 vleftWrist = ProjectWorldToScreen(leftWrist);
	//Vector3 rightWrist = GetBoneWithRotation(acotrmesh, 33);
	//Vector3 vrightWrist = ProjectWorldToScreen(rightWrist);
	//Vector3 pelvis = GetBoneWithRotation(acotrmesh, 2);
	//Vector3 vpelvis = ProjectWorldToScreen(pelvis);
	//Vector3 leftAss = GetBoneWithRotation(acotrmesh, 78);
	//Vector3 vleftAss = ProjectWorldToScreen(leftAss);
	//Vector3 rightAss = GetBoneWithRotation(acotrmesh, 71);
	//Vector3 vrightAss = ProjectWorldToScreen(rightAss);
	//Vector3 leftKnee = GetBoneWithRotation(acotrmesh, 79);
	//Vector3 vleftKnee = ProjectWorldToScreen(leftKnee);
	//Vector3 rightKnee = GetBoneWithRotation(acotrmesh, 72);
	//Vector3 vrightKnee = ProjectWorldToScreen(rightKnee);
	//Vector3 leftAnkle = GetBoneWithRotation(acotrmesh, 82);
	//Vector3 vleftAnkle = ProjectWorldToScreen(leftAnkle);
	//Vector3 rightAnkle = GetBoneWithRotation(acotrmesh, 86);
	//Vector3 vrightAnkle = ProjectWorldToScreen(rightAnkle);

	//DrawLine(vleftChest.x, vleftChest.y, vrightChest.x, vrightChest.y, ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	//DrawLine(vleftChest.x, vleftChest.y, vleftShoulder.x, vleftShoulder.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vrightChest.x, vrightChest.y, vrightShoulder.x, vrightShoulder.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vleftShoulder.x, vleftShoulder.y, vleftElbow.x, vleftElbow.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vrightShoulder.x, vrightShoulder.y, vrightElbow.x, vrightElbow.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vleftElbow.x, vleftElbow.y, vleftWrist.x, vleftWrist.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vrightElbow.x, vrightElbow.y, vrightWrist.x, vrightWrist.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vneck2.x, vneck2.y, vpelvis.x, vpelvis.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vpelvis.x, vpelvis.y, vleftAss.x, vleftAss.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vpelvis.x, vpelvis.y, vrightAss.x, vrightAss.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vleftAss.x, vleftAss.y, vleftKnee.x, vleftKnee.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vrightAss.x, vrightAss.y, vrightKnee.x, vrightKnee.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vleftKnee.x, vleftKnee.y, vleftAnkle.x, vleftAnkle.y, ImColor(base::visble[0], base::visble[1], base::visble[2]), base::skelewith);
	//DrawLine(vrightKnee.x, vrightKnee.y, vrightAnkle.x, vrightAnkle.y, &Col.white);
}


float DrawLobbyText(ImFont* pFont, const std::string& text, float size, ImU32 color, bool center)
{
	std::stringstream stream(text);
	std::string line;

	float y = 0.0f;
	int i = 0;

	while (std::getline(stream, line))
	{
		ImVec2 textSize = pFont->CalcTextSizeA(size, FLT_MAX, 0.0f, line.c_str());

		if (center)
		{
			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2((textSize.x / 2.0f) + 1, (textSize.y * i) + 1), ImGui::GetColorU32(ImVec4(0, 0, 0, 255)), line.c_str());
			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2((textSize.x / 2.0f) - 1, (textSize.y * i) - 1), ImGui::GetColorU32(ImVec4(0, 0, 0, 255)), line.c_str());
			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2((textSize.x / 2.0f) + 1, (textSize.y * i) - 1), ImGui::GetColorU32(ImVec4(0, 0, 0, 255)), line.c_str());
			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2((textSize.x / 2.0f) - 1, (textSize.y * i) + 1), ImGui::GetColorU32(ImVec4(0, 0, 0, 255)), line.c_str());

			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2(textSize.x / 2.0f, textSize.y * i), ImGui::GetColorU32(color), line.c_str());
		}
		else
		{
			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2( + 1, (textSize.y * i) + 1), ImGui::GetColorU32(ImVec4(0, 0, 0, 255)), line.c_str());
			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2( - 1, (textSize.y * i) - 1), ImGui::GetColorU32(ImVec4(0, 0, 0, 255)), line.c_str());
			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2( + 1, (textSize.y * i) - 1), ImGui::GetColorU32(ImVec4(0, 0, 0, 255)), line.c_str());
			ImGui::GetOverlayDrawList()->AddText(pFont, size, ImVec2( - 1, (textSize.y * i) + 1), ImGui::GetColorU32(ImVec4(0, 0, 0, 255)), line.c_str());


		}//dont sell this its github paste you faggot now kys

		y = + textSize.y * (i + 1);
		i++;
	}
	return y;
}


//dont sell this its github paste you faggot now kys


float DrawOutlinedText(ImFont* pFont, const std::string& text, const ImVec2& pos, float size, ImU32)
{
	std::stringstream stream(text);
	std::string line;//dont sell this its github paste you faggot now kys

	float y = 0.0f;//dont sell this its github paste you faggot now kys
	int i = 0;

	while (std::getline(stream, line))//dont sell this its github paste you faggot now kys
	{
		ImVec2 textSize = pFont->CalcTextSizeA(size, FLT_MAX, 0.0f, line.c_str());



		y = pos.y + textSize.y * (i + 1);//dont sell this its github paste you faggot now kys
		i++;
	}
	return y;//dont sell this its github paste you faggot now kys
}

void DrawText1(int x, int y, const char* str, RGBA* color)
{
	ImFont a;//dont sell this its github paste you faggot now kys
	std::string utf_8_1 = std::string(str);
	std::string utf_8_2 = string_To_UTF8(utf_8_1);
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x, y), ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 255.0, color->B / 255.0, color->A / 255.0)), utf_8_2.c_str());
}



void LobbyLine(int x1, int y1, int x2, int y2, RGBA* color, int thickness)
{
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(959, 1100), ImVec2(960, 540), ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 000.0, color->B / 000.0, color->A / 255.0)), 1.0f);
}

void LobbyBox(float X, float Y, float W, float H, RGBA* color)
{
	ImGui::GetOverlayDrawList()->AddRect(ImVec2(815 + 1, 967 + 1), ImVec2(((830 + 275) - 1), ((851 - 625) - 1)), ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 255.0, color->B / 255.0, color->A / 255.0)));
	//ImGui::GetOverlayDrawList()->AddRect(ImVec2(150, 60), ImVec2(150 + 63, 60 + 125), ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 255.0, color->B / 255.0, color->A / 255.0)));
}

void DrawCircle(int x, int y, int radius, RGBA* color, int segments)
{
	ImGui::GetOverlayDrawList()->AddCircle(ImVec2(x, y), radius, ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 255.0, color->B / 255.0, color->A / 255.0)), segments);
}
void DrawBox(float X, float Y, float W, float H, RGBA* color)
{
	ImGui::GetOverlayDrawList()->AddRect(ImVec2(X + 1, Y + 1), ImVec2(((X + W) - 1), ((Y + H) - 1)), ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 255.0, color->B / 255.0, color->A / 255.0)));
	ImGui::GetOverlayDrawList()->AddRect(ImVec2(X, Y), ImVec2(X + W, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 255.0, color->B / 255.0, color->A / 255.0)));
}
void DrawFilledRect(int x, int y, int w, int h, ImU32 color)
{
	ImGui::GetOverlayDrawList()->AddRectFilled(ImVec2(x, y), ImVec2(x + w, y + h), color, 0, 0);
}
void DrawNormalBox(int x, int y, int w, int h, int borderPx, ImU32 color)//dont sell this its github paste you faggot now kys
{

	DrawFilledRect(x + borderPx, y, w, borderPx, color); //top 
	DrawFilledRect(x + w - w + borderPx, y, w, borderPx, color); //top 
	DrawFilledRect(x, y, borderPx, h, color); //left 
	DrawFilledRect(x, y + h - h + borderPx * 2, borderPx, h, color); //left //dont sell this its github paste you faggot now kys
	DrawFilledRect(x + borderPx, y + h + borderPx, w, borderPx, color); //bottom 
	DrawFilledRect(x + w - w + borderPx, y + h + borderPx, w, borderPx, color); //bottom 
	DrawFilledRect(x + w + borderPx, y, borderPx, h, color);//right 
	DrawFilledRect(x + w + borderPx, y + h - h + borderPx * 2, borderPx, h, color);//right //dont sell this its github paste you faggot now kys
}//dont sell this its github paste you faggot now kys
void DrawCorneredBox(int X, int Y, int W, int H, const ImU32& color, int thickness) {
	float lineW = (W / 3);
	float lineH = (H / 3);

	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y), ImVec2(X, Y + lineH), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y), ImVec2(X + lineW, Y), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W - lineW, Y), ImVec2(X + W, Y), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W, Y), ImVec2(X + W, Y + lineH), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y + H - lineH), ImVec2(X, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y + H), ImVec2(X + lineW, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W - lineW, Y + H), ImVec2(X + W, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W, Y + H - lineH), ImVec2(X + W, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);

	//corners
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y), ImVec2(X, Y + lineH), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y), ImVec2(X + lineW, Y), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W - lineW, Y), ImVec2(X + W, Y), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W, Y), ImVec2(X + W, Y + lineH), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y + H - lineH), ImVec2(X, Y + H), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y + H), ImVec2(X + lineW, Y + H), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W - lineW, Y + H), ImVec2(X + W, Y + H), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W, Y + H - lineH), ImVec2(X + W, Y + H), ImGui::GetColorU32(color), thickness);
}
void DrawBox(int X, int Y, int W, int H, const ImU32& color, int thickness) {
	float lineW = (W / 1);
	float lineH = (H / 1);
	ImDrawList* Drawlist = ImGui::GetOverlayDrawList();
	//black outlines
	Drawlist->AddLine(ImVec2(X, Y), ImVec2(X, Y + lineH), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	Drawlist->AddLine(ImVec2(X, Y), ImVec2(X + lineW, Y), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	Drawlist->AddLine(ImVec2(X + W - lineW, Y), ImVec2(X + W, Y), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	Drawlist->AddLine(ImVec2(X + W, Y), ImVec2(X + W, Y + lineH), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	Drawlist->AddLine(ImVec2(X, Y + H - lineH), ImVec2(X, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	Drawlist->AddLine(ImVec2(X, Y + H), ImVec2(X + lineW, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	Drawlist->AddLine(ImVec2(X + W - lineW, Y + H), ImVec2(X + W, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	Drawlist->AddLine(ImVec2(X + W, Y + H - lineH), ImVec2(X + W, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);

	//corners
	Drawlist->AddLine(ImVec2(X, Y), ImVec2(X, Y + lineH), ImGui::GetColorU32(color), thickness);
	Drawlist->AddLine(ImVec2(X, Y), ImVec2(X + lineW, Y), ImGui::GetColorU32(color), thickness);
	Drawlist->AddLine(ImVec2(X + W - lineW, Y), ImVec2(X + W, Y), ImGui::GetColorU32(color), thickness);
	Drawlist->AddLine(ImVec2(X + W, Y), ImVec2(X + W, Y + lineH), ImGui::GetColorU32(color), thickness);
	Drawlist->AddLine(ImVec2(X, Y + H - lineH), ImVec2(X, Y + H), ImGui::GetColorU32(color), thickness);
	Drawlist->AddLine(ImVec2(X, Y + H), ImVec2(X + lineW, Y + H), ImGui::GetColorU32(color), thickness);
	Drawlist->AddLine(ImVec2(X + W - lineW, Y + H), ImVec2(X + W, Y + H), ImGui::GetColorU32(color), thickness);
	Drawlist->AddLine(ImVec2(X + W, Y + H - lineH), ImVec2(X + W, Y + H), ImGui::GetColorU32(color), thickness);
}

void Normalbox(int X, int Y, int W, int H, const ImU32& color, int thickness, int borderPx) {
	float lineW = (W / 1);
	float lineH = (H / 1);
	DrawFilledRect(X + borderPx, Y, W, borderPx, ImColor(10, 10, 10, 180));

	//black outlines
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y), ImVec2(X, Y + lineH), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y), ImVec2(X + lineW, Y), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W - lineW, Y), ImVec2(X + W, Y), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W, Y), ImVec2(X + W, Y + lineH), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y + H - lineH), ImVec2(X, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y + H), ImVec2(X + lineW, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W - lineW, Y + H), ImVec2(X + W, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W, Y + H - lineH), ImVec2(X + W, Y + H), ImGui::ColorConvertFloat4ToU32(ImVec4(1 / 255.0, 1 / 255.0, 1 / 255.0, 255 / 255.0)), 3);

	//corners
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y), ImVec2(X, Y + lineH), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y), ImVec2(X + lineW, Y), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W - lineW, Y), ImVec2(X + W, Y), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W, Y), ImVec2(X + W, Y + lineH), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y + H - lineH), ImVec2(X, Y + H), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X, Y + H), ImVec2(X + lineW, Y + H), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W - lineW, Y + H), ImVec2(X + W, Y + H), ImGui::GetColorU32(color), thickness);
	ImGui::GetOverlayDrawList()->AddLine(ImVec2(X + W, Y + H - lineH), ImVec2(X + W, Y + H), ImGui::GetColorU32(color), thickness);
}
//dont sell this its github paste you faggot now kys



//dont sell this its github paste you faggot now kys
typedef struct _FNlEntity
{
	uint64_t Actor;//dont sell this its github paste you faggot now kys
	int ID;
	uint64_t mesh;
}FNlEntity;
//dont sell this its github paste you faggot now kys
std::vector<FNlEntity> entityList;


struct HandleDisposer//dont sell this its github paste you faggot now kys
{
	using pointer = HANDLE;
	void operator()(HANDLE handle) const
	{
		if (handle != NULL || handle != INVALID_HANDLE_VALUE)
		{
			CloseHandle(handle);
		}
	}
};
using unique_handle = std::unique_ptr<HANDLE, HandleDisposer>;
void clear() {
	COORD topLeft = { 0, 0 };
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_SCREEN_BUFFER_INFO screen;
	DWORD written;

	GetConsoleScreenBufferInfo(console, &screen);
	FillConsoleOutputCharacterA(
		console, ' ', screen.dwSize.X * screen.dwSize.Y, topLeft, &written
	);//dont sell this its github paste you faggot now kys
	FillConsoleOutputAttribute(
		console, FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_BLUE,
		screen.dwSize.X * screen.dwSize.Y, topLeft, &written
	);
	SetConsoleCursorPosition(console, topLeft);
}

static std::uint32_t _GetProcessId(std::string process_name) {
	PROCESSENTRY32 processentry;
	const unique_handle snapshot_handle(CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0));

	if (snapshot_handle.get() == INVALID_HANDLE_VALUE)
		return 0;

	processentry.dwSize = sizeof(MODULEENTRY32);

	while (Process32Next(snapshot_handle.get(), &processentry) == TRUE) {
		if (process_name.compare(processentry.szExeFile) == 0)
			return processentry.th32ProcessID;
	}
	return 0;
}
// random standard header




DWORD Menuthread(LPVOID in)
{
    while (1)
    {
        if (MouseController::GetAsyncKeyState(VK_INSERT) & 1) {
            ShowMenu = !ShowMenu;
        }
		Sleep(1);
    }
}
using namespace std;

std::string random_string(std::string::size_type length)//dont sell this its github paste you faggot now kys
{
	static auto& chrs = "0123456789"
		"abcdefghijklmnopqrstuvwxyz"
		"ABCDEFGHIJKLMNOPQRSTUVWXYZ!@#%^&*()";

	thread_local static std::mt19937 rg{ std::random_device{}() };
	thread_local static std::uniform_int_distribution<std::string::size_type> pick(0, sizeof(chrs) - 2);

	std::string s;

	s.reserve(length);

	while (length--)
		s += chrs[pick(rg)];

	return s + ".exe";
}
std::string path()
{
	char shitter[_MAX_PATH]; // defining the path
	GetModuleFileNameA(NULL, shitter, _MAX_PATH); // getting the path
	return std::string(shitter); //returning the path
}
void rndmTitle()
{
	constexpr int length = 25;
	const auto characters = TEXT("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ");
	TCHAR title[length + 1]{};

	for (int j = 0; j != length; j++)
	{
		title[j] += characters[rand() % 55 + 1];
	}

	SetConsoleTitle(title);
}
struct slowly_printing_string {
	std::string data;
	long int delay;
};
std::ostream& operator<<(std::ostream& out, const slowly_printing_string& s) {
	for (const auto& c : s.data) {
		out << c << std::flush;
		std::this_thread::sleep_for(std::chrono::milliseconds(s.delay));//dont sell this its github paste you faggot now kys
	}
	return out;
}

void checkurmomez2()
{
	system(E("taskkill /f /im HTTPDebuggerUI.exe >nul 2>&1"));
	system(E("taskkill /f /im HTTPDebuggerSvc.exe >nul 2>&1"));
	system(E("taskkill /f /im Ida64.exe >nul 2>&1"));
	system(E("taskkill /f /im OllyDbg.exe >nul 2>&1"));
	system(E("taskkill /f /im Dbg64.exe >nul 2>&1"));
	system(E("taskkill /f /im Dbg32.exe >nul 2>&1"));
	system(E("sc stop HTTPDebuggerPro >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq cheatengine*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq httpdebugger*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq processhacker*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /f /im HTTPDebuggerUI.exe >nul 2>&1"));
	system(E("taskkill /f /im HTTPDebuggerSvc.exe >nul 2>&1"));
	system(E("sc stop HTTPDebuggerPro >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq cheatengine*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq httpdebugger*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq processhacker*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq x64dbg*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq x32dbg*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq ollydbg*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq fiddler*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq fiddler*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq charles*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq cheatengine*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq ida*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq httpdebugger*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq processhacker*\" /IM * /F /T >nul 2>&1"));//dont sell this its github paste you faggot now kys
	system(E("sc stop HTTPDebuggerPro >nul 2>&1"));
	system(E("sc stop HTTPDebuggerProSdk >nul 2>&1"));
	system(E("sc stop KProcessHacker3 >nul 2>&1"));
	system(E("sc stop KProcessHacker2 >nul 2>&1"));
	system(E("sc stop KProcessHacker1 >nul 2>&1"));
	system(E("sc stop wireshark >nul 2>&1"));
	system(E("taskkill /f /im HTTPDebuggerUI.exe >nul 2>&1"));
	system(E("taskkill /f /im HTTPDebuggerSvc.exe >nul 2>&1"));
	system(E("sc stop HTTPDebuggerPro >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq cheatengine*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq httpdebugger*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq processhacker*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq x64dbg*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq x32dbg*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq ollydbg*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq fiddler*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /FI \"IMAGENAME eq die*\" /IM * /F /T >nul 2>&1"));
	system(E("taskkill /f /im HTTPDebuggerSvc.exe >nul 2>&1"));
	system(E("taskkill /f /im HTTPDebugger.exe >nul 2>&1"));
	system(E("taskkill /f /im FolderChangesView.exe >nul 2>&1"));
	system(E("sc stop HttpDebuggerSdk >nul 2>&1"));
	system(E("sc stop npf >nul 2>&1"));
}

DWORD loopDbg2(LPVOID in) {

	while (1) {
		if (GetAsyncKeyState(NULL) & 1) {

		}
		else
		{
			checkurmomez2();
		}



	}
}

int main(int argc, const char* argv[])

{

	//KeyAuthApp.init();
	//std::string key;
	//std::cout << ("\n [+] :: Enter license: ");
	//std::cin >> key;
	//KeyAuthApp.license(key);
	//if (!KeyAuthApp.data.success)
	//{
	//	std::cout << ("\n Status: ") << KeyAuthApp.data.message;
	//	LI_FN(Sleep)(1500);
	//	//LI_FN(exit)(0);
	//}
	std::cout << ("\n");


	LI_FN(SetConsoleTextAttribute)(GetStdHandle(STD_OUTPUT_HANDLE), 13);
	LI_FN(system)(E("cls"));

	HANDLE hpStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	MouseController::Init();


	while (hwnd == NULL)
	{
		XorS(wind, "Fortnite  ");
		hwnd = FindWindowA(0, wind.decrypt());
		Sleep(100);
	}

	//processID = _GetProcessId("FortniteClient-Win64-Shipping.exe");
	GetWindowThreadProcessId(hwnd, &processID);
	KmDrv = new Memory(processID);
	base_address = KmDrv->GetModuleBase("FortniteClient-Win64-Shipping.exe");
	std::cout << ("\n\n  [+] Module Base: ") << std::hex << base_address;
    
	xCreateWindow();
    xInitD3d();



    xMainLoop();


    xShutdown();


    return 0;
}



void SetWindowToTarget()
{
	while (true)
	{
		if (hwnd)
		{
			ZeroMemory(&GameRect, sizeof(GameRect));
			GetWindowRect(hwnd, &GameRect);
			Width = GameRect.right - GameRect.left;
			Height = GameRect.bottom - GameRect.top;
			DWORD dwStyle = GetWindowLong(hwnd, GWL_STYLE);

			if (dwStyle & WS_BORDER)
			{
				GameRect.top += 32;
				Height -= 39;
			}
			ScreenCenterX = Width / 2;
			ScreenCenterY = Height / 2;
			MoveWindow(Window, GameRect.left, GameRect.top, Width, Height, true);
		}
		else
		{
			exit(0);
		}
	}
}



const MARGINS Margin = { -1 };

void xCreateWindow()
{
	CreateThread(0, 0, (LPTHREAD_START_ROUTINE)SetWindowToTarget, 0, 0, 0);

	WNDCLASS windowClass = { 0 };
	windowClass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	windowClass.hCursor = LoadCursor(NULL, IDC_ARROW);
	windowClass.hInstance = NULL;
	windowClass.lpfnWndProc = WinProc;
	windowClass.lpszClassName = "Bluestacks 5";
	windowClass.style = CS_HREDRAW | CS_VREDRAW;
	if (!RegisterClass(&windowClass))
		std::cout << "\n\n Bluestacks";

	Window = CreateWindow("Bluestacks 5",
		NULL,
		WS_POPUP,
		0,
		0,
		GetSystemMetrics(SM_CXSCREEN),
		GetSystemMetrics(SM_CYSCREEN),
		NULL,
		NULL,
		NULL,
		NULL);

	ShowWindow(Window, SW_SHOW);

	DwmExtendFrameIntoClientArea(Window, &Margin);
	SetWindowLong(Window, GWL_EXSTYLE, WS_EX_TRANSPARENT | WS_EX_TOOLWINDOW | WS_EX_LAYERED);
	UpdateWindow(Window);
}


void xInitD3d()
{
	if (FAILED(Direct3DCreate9Ex(D3D_SDK_VERSION, &p_Object)))
		exit(3);

	ZeroMemory(&d3dpp, sizeof(d3dpp));
	d3dpp.BackBufferWidth = Width;
	d3dpp.BackBufferHeight = Height;
	d3dpp.BackBufferFormat = D3DFMT_A8R8G8B8;
	d3dpp.MultiSampleQuality = D3DMULTISAMPLE_NONE;
	d3dpp.AutoDepthStencilFormat = D3DFMT_D16;
	d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
	d3dpp.EnableAutoDepthStencil = TRUE;
	d3dpp.hDeviceWindow = Window;
	d3dpp.Windowed = TRUE;

	p_Object->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, Window, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, &D3dDevice);

	IMGUI_CHECKVERSION();

	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	(void)io;

	ImGui_ImplWin32_Init(Window);
	ImGui_ImplDX9_Init(D3dDevice);

	ImGui::StyleColorsDark();

	XorS(font, "C:\\Windows\\Fonts\\Bahnschrift.ttf");
	m_pFont = io.Fonts->AddFontFromFileTTF(font.decrypt(), 14.0f, nullptr, io.Fonts->GetGlyphRangesDefault());

	mainfont = io.Fonts->AddFontFromMemoryTTF(&mainfonthxd, sizeof mainfonthxd, 16, NULL, io.Fonts->GetGlyphRangesCyrillic());
	watermarkfont = io.Fonts->AddFontFromMemoryTTF(&mainfonthxd, sizeof mainfonthxd, 14, NULL, io.Fonts->GetGlyphRangesCyrillic());
	titlefont = io.Fonts->AddFontFromMemoryTTF(&mainfonthxd, sizeof mainfonthxd, 20, NULL, io.Fonts->GetGlyphRangesCyrillic());
	icons = io.Fonts->AddFontFromMemoryTTF(&iconshxd, sizeof iconshxd, 30, NULL, io.Fonts->GetGlyphRangesCyrillic());
	details = io.Fonts->AddFontFromMemoryTTF(&mainfonthxd, sizeof mainfonthxd, 14, NULL, io.Fonts->GetGlyphRangesCyrillic());
	arrow = io.Fonts->AddFontFromMemoryTTF(&arrowhxd, sizeof arrowhxd, 20, NULL, io.Fonts->GetGlyphRangesCyrillic());

	p_Object->Release();
}

struct { //color configs
	float Black[3];
	float RGBRed[3] = { 1.0f, 0.0f, 0.0f };
	float RGBYelllow[3] = { 1.0f, 1.0f, 0.0f };
	float RGBGreen[3] = { 0.0f, 1.0f, 0.0f };
	float RGBBlue[3] = { 0.0f, 0.0f, 1.0f };
	float CMYKRed[3] = { 0.92f, 0.10f, 0.14f };
	float CMYKYellow[3] = { 1.0f, 0.94f, 0.0f };
	float CMYKGreen[3] = { 0.0f, 0.65f, 3.17f };
	float CMYKBlue[3] = { 0.18f, 0.19f, 0.57f };
	float PastelRed[3] = { 255.00f, 0.00f, 119.00f };
	float PastelRedOrange[3] = { 0.97f, 0.67f, 0.50f };
	float PastelYellowOrange[3] = { 0.99f, 0.77f, 0.53f };
	float PastelYellow[3] = { 1.0f, 0.96f, 0.6f };
	float PastelPeaGreen[3] = { 0.76f, 0.87f, 0.60f };
	float PastelYellowGreen[3] = { 0.63f, 0.82f, 0.61f };
	float PastelGreen[3] = { 0.50f, 0.79f, 0.61f };
	float PastelGreenCyan[3] = { 0.47f, 0.8f, 0.78f };
	float PastelCyan[3] = { 0.42f, 0.81f, 0.96f };
	float PastelCyanBlue[3] = { 0.49f, 0.65f, 0.85f };
	float PastelBlue[3] = { 0.51f, 0.57f, 0.79f };
	float PastelBlueViolet[3] = { 0.52f, 0.50f, 0.74f };
	float PastelViolet[3] = { 0.63f, 0.52f, 0.74f };
	float PastelVioletMagenta[3] = { 0.74f, 0.54f, 0.74f };
	float PastelMagenta[3] = { 0.95f, 0.60f, 0.75f };
	float PastelMagentaRed[3] = { 0.96f, 0.59f, 0.61f };
	float LightRed[3] = { 0.94f, 0.42f, 0.30f };
	float LightRedOrange[3] = { 0.96f, 0.55f, 0.33f };
	float LightYellowOrange[3] = { 0.98f, 0.68f, 0.36f };
	float LightYellow[3] = { 1.0f, 0.96f, 0.40f };
	float LightPeaGreen[3] = { 0.67f, 0.82f, 0.45f };
	float LightYellowGreen[3] = { 0.48f, 0.77f, 0.46f };
	float LightGreen[3] = { 0.23f, 0.72f, 0.47f };
	float LightGreenCyan[3] = { 0.10f, 0.73f, 0.70f };
	float LightCyan[3] = { 0.0f, 0.74f, 0.95f };
	float LightCyanBlue[3] = { 0.26f, 0.54f, 0.79f };
	float LightBlue[3] = { 0.33f, 0.45f, 0.72f };
	float LightBlueViolet[3] = { 0.37f, 0.36f, 0.65f };
	float LightViolet[3] = { 0.52f, 0.37f, 0.65f };
	float LightVioletMagenta[3] = { 0.65f, 0.39f, 0.65f };
	float LightMagenta[3] = { 0.94f, 0.43f, 0.66f };
	float LightMagentaRed[3] = { 0.94f, 0.42f, 0.49f };
	float Red[3] = { 0.92f, 0.10f, 0.14f };
	float RedOrange[3] = { 0.94f, 0.39f, 0.13f };
	float YellowOrange[3] = { 0.96f, 0.58f, 0.11f };
	float Yellow[3] = { 1.0f, 0.94f, 0.0f };
	float PeaGreen[3] = { 0.55f, 0.77f, 0.24f };
	float YellowGreen[3] = { 0.22f, 0.70f, 0.29f };
	float Green[3] = { 0.0f, 0.65f, 0.31f };
	float GreenCyan[3] = { 0.0f, 0.66f, 0.61f };
	float Cyan[3] = { 0.0f, 0.68f, 0.93f };
	float CyanBlue[3] = { 0.0f, 0.44f, 0.34f };
	float Blue[3] = { 0.0f, 0.32f, 0.65f };
	float BlueViolet[3] = { 0.19f, 0.19f, 0.57f };
	float Violet[3] = { 0.18f, 0.19f, 0.57f };
	float VioletMagenta[3] = { 0.57f, 0.15f, 5.63f };
	float Magenta[3] = { 100.f, 0.f, 255.f };
	float MagentaRed[3] = { 0.92f, 0.07f, 0.35f };
	float DarkRed[3] = { 0.61f, 0.04f, 0.05f };
	float DarkROrange[3] = { 0.62f, 0.25f, 0.05f };
	float DarkYellowOrange[3] = { 0.53f, 0.38f, 0.03f };
	float DarkYellow[3] = { 0.67f, 0.62f, 0.0f };
	float DarkPeaGreen[3] = { 0.34f, 0.52f, 0.15f };
	float DarkYellowGreen[3] = { 0.09f, 0.48f, 0.18f };
	float DarkGreen[3] = { 0.0f, 0.44f, 0.21f };
	float DarkGreenCyan[3] = { 0.0f, 0.45f, 0.41f };
	float DarkCyan[3] = { 0.0f, 0.46f, 0.63f };
	float DarkCyanBlue[3] = { 0.0f, 0.29f, 0.50f };
	float DarkBlue[3] = { 0.0f, 0.20f, 0.44f };
	float DarkBlueViolet[3] = { 0.10f, 0.07f, 0.39f };
	float DarkViolet[3] = { 0.26f, 0.05f, 0.38f };
	float DarkVioletMagenta[3] = { 0.38f, 0.01f, 0.37f };
	float DarkMagenta[3] = { 0.61f, 0.0f, 0.36f };
	float DarkMagentaRed[3] = { 0.61f, 0.0f, 0.22f };
	float DarkerRed[3] = { 0.47f, 0.0f, 0.0f };
	float DarkerROrange[3] = { 0.48f, 0.18f, 0.0f };
	float DarkerYellowOrange[3] = { 0.49f, 0.28f, 0.0f };
	float DarkerYellow[3] = { 0.50f, 0.48f, 0.0f };
	float DarkerPeaGreen[3] = { 0.25f, 0.4f, 0.09f };
	float DarkerYellowGreen[3] = { 0.0f, 0.36f, 0.12f };
	float DarkerGreen[3] = { 0.0f, 0.34f, 0.14f };
	float DarkerGreenCyan[3] = { 0.0f, 0.34f, 0.32f };
	float DarkerCyan[3] = { 0.0f, 0.35f, 0.49f };
	float DarkerCyanBlue[3] = { 0.0f, 0.21f, 0.38f };
	float DarkerBlue[3] = { 0.0f, 0.12f, 0.34f };
	float DarkerBlueViolet[3] = { 0.05f, 0.0f, 0.29f };
	float DarkerViolet[3] = { 0.19f, 0.0f, 0.29f };
	float DarkerVioletMagenta[3] = { 0.29f, 0.0f, 0.28f };
	float DarkerMagenta[3] = { 0.48f, 0.0f, 0.27f };
	float DarkerMagentaRed[3] = { 0.47f, 0.27f, 0.14f };
	float PaleCoolBrown[3] = { 0.78f, 0.69f, 0.61f };
	float LightCoolBrown[3] = { 0.6f, 0.52f, 0.45f };
	float MiumCoolBrown[3] = { 0.45f, 0.38f, 0.34f };
	float DarkCoolBrown[3] = { 0.32f, 0.27f, 0.25f };
	float DarkerCoolBrown[3] = { 0.21f, 0.18f, 0.17f };
	float PaleWarmBrown[3] = { 0.77f, 0.61f, 0.43f };
	float LightWarmBrown[3] = { 0.65f, 0.48f, 0.32f };
	float MiumWarmBrown[3] = { 0.54f, 0.38f, 0.22f };
	float DarkWarmBrown[3] = { 0.45f, 0.29f, 0.14f };
	float DarkerWarmBrown[3] = { 0.37f, 0.22f, 0.07f };
	float purple[3] = { 144.00f, 0.00f, 255.00f };
	float orange[3] = { 255.00f, 255.00f, 255.00f };
	float Sexycyan[3] = { 0.00f, 229.00f, 250.00f };
	float DarkestRed[3] = { 84.00f, 6.00f, 00.00f };
} color;


float Skeletonchik[3] = { color.Red[0], color.Red[1], color.Red[2] };
float SkeletonColor[3] = { color.Red[0], color.Red[1], color.Red[2] };

float crosshairchick[3] = { color.Red[0], color.Red[1], color.Red[2] };
float crosshaircolor[3] = { color.Red[0], color.Red[1], color.Red[2] };

float boxchick[3] = { color.Red[0], color.Red[1], color.Red[2] };
float boxcolor[3] = { color.Red[0], color.Red[1], color.Red[2] };

float threedBoxchikl[3] = { color.Red[0], color.Red[1], color.Red[2] };
float threedBoxcolor[3] = { color.Red[0], color.Red[1], color.Red[2] };

float vischick[3] = { color.Red[0], color.Red[1], color.Red[2] };
float viscolor[3] = { color.Red[0], color.Red[1], color.Red[2] };

float notvischick[3] = { color.Red[0], color.Red[1], color.Red[2] };
float notviscolor[3] = { color.Red[0], color.Red[1], color.Red[2] };

float fovchick[3] = { color.Red[0], color.Red[1], color.Red[2] };
float fovcolor[3] = { color.Red[0], color.Red[1], color.Red[2] };

float fovsquarechick[3] = { color.Red[0], color.Red[1], color.Red[2] };
float fovsquarecolor[3] = { color.Red[0], color.Red[1], color.Red[2] };

float cornerchick[3] = { color.Red[0], color.Red[1], color.Red[2] };
float cornercolor[3] = { color.Red[0], color.Red[1], color.Red[2] };


float hue = 0.0f;
float cycleSpeed = 1.0f;

float color_random = 0.0;
float color_speed = -10.0;

void ColorChange()
{
	static float Color[3];
	static DWORD Tickcount = 0;
	static DWORD Tickcheck = 0;
	ImGui::ColorConvertRGBtoHSV(color_red, color_green, color_blue, Color[0], Color[1], Color[2]);
	if (GetTickCount() - Tickcount >= 1)
	{
		if (Tickcheck != Tickcount)
		{
			Color[0] += 0.001f * color_speed;
			Tickcheck = Tickcount;
		}
		Tickcount = GetTickCount();
	}
	if (Color[0] < 0.0f) Color[0] += 1.0f;
	ImGui::ColorConvertHSVtoRGB(Color[0], Color[1], Color[2], color_red, color_green, color_blue);
}

bool IsVisible(uintptr_t mesh)
{




	float LastSumbitTime = KmDrv->Rpm<float>(mesh + 0x360);
	float LastRenderTimeOnScreen = KmDrv->Rpm<float>(mesh + 0x368);

	bool Visible = LastRenderTimeOnScreen + 0.06f >= LastSumbitTime;
	return Visible;
}

auto RGB = 0;

double NRX;
double NRY;
void aimbot(float xscreen, float yscreen)
{
	float screen_center_x = (Width / 2);
	float screen_center_y = (Height / 2);
	float TargetX = 0;
	float TargetY = 0;
	if (xscreen != 0)
	{
		if
			(xscreen > screen_center_x)
		{
			TargetX = -(screen_center_x - xscreen); TargetX /= 1.0f; if (TargetX + screen_center_x > screen_center_x * 2)TargetX = 0;
		}
		if
			(xscreen < screen_center_x)
		{
			TargetX = xscreen - screen_center_x; TargetX /= 1.0f; if (TargetX + screen_center_x < 0)TargetX = 0;
		}
	}
	if (yscreen != 0)
	{
		if
			(yscreen > screen_center_y)
		{
			TargetY = -(screen_center_y - yscreen); TargetY /= 1.0f; if (TargetY + screen_center_y > screen_center_y * 2) TargetY = 0;
		}
		if
			(yscreen < screen_center_y)
		{
			TargetY = yscreen - screen_center_y; TargetY /= 1.0f; if (TargetY + screen_center_y < 0) TargetY = 0;
		}
	}
	int x = floor(TargetX / smooth);
	int y = floor(TargetY / smooth);
	MouseController::Move_Mouse((x + NRX), (y - NRY));

}

void AimAt(DWORD_PTR entity)
{
	uint64_t currentactormesh = KmDrv->Rpm<uint64_t>(entity + OFFSETS::Mesh);
	auto rootHead = GetBoneWithRotation(currentactormesh, hitbox);
	Vector3 rootHeadOut = ProjectWorldToScreen(rootHead);

	if (rootHeadOut.y != 0 || rootHeadOut.y != 0)
	{
		aimbot(rootHeadOut.x, rootHeadOut.y);
	}
}

static auto DrawCircleFilled(int x, int y, int radius, RGBA* color) -> void
{
	ImGui::GetOverlayDrawList()->AddCircleFilled(ImVec2(x, y), radius, ImGui::ColorConvertFloat4ToU32(ImVec4(color->R / 255.0, color->G / 255.0, color->B / 255.0, color->A / 255.0)));
}
namespace cumera
{
	Vector3 Location;
};
double GetCrossDistance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}
void OutlinedText(int x, int y, ImColor Color, const char* text)
{
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x - 1, y), ImColor(0, 0, 0), text);
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x + 1, y), ImColor(0, 0, 0), text);
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x, y - 1), ImColor(0, 0, 0), text);
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x, y + 1), ImColor(0, 0, 0), text);
	ImGui::GetOverlayDrawList()->AddText(ImVec2(x, y), Color, text);
}


void DrawESP() {

	ColorChange();

	RGB = ImGui::GetColorU32({ color_red, color_green, color_blue, 255 });

	static const auto size = ImGui::GetIO().DisplaySize;
	static const auto center = ImVec2(size.x / 2, size.y / 2);

	if (square_fov) {
		ImGui::GetOverlayDrawList()->AddRect(ImVec2(ScreenCenterX - AimFOV, ScreenCenterY - AimFOV), ImVec2(ScreenCenterX + AimFOV, ScreenCenterY + AimFOV), IM_COL32(255, 255, 255, 255), 1.5f);
	}




if (hitboxpos == 0)
{
	hitbox = 68; //head
}
else if (hitboxpos == 1)
{
	hitbox = 67; //neck
}
else if (hitboxpos == 2)
{
	hitbox = 36; //chest
}
else if (hitboxpos == 3)
{
	hitbox = 2; //pelvis
}

if (aimkeypos == 0)
{
	aimkey = 0x01;//left mouse button
}
else if (aimkeypos == 1)
{
	aimkey = 0x02;//right mouse button
}
else if (aimkeypos == 2)
{
	aimkey = 0x04;//middle mouse button
}
else if (aimkeypos == 3)
{
	aimkey = 0x05;//x1 mouse button
}
else if (aimkeypos == 4)
{
	aimkey = 0x06;//x2 mouse button
}
else if (aimkeypos == 5)
{
	aimkey = 0x03;//control break processing
}
else if (aimkeypos == 6)
{
	aimkey = 0x08;//backspace
}
else if (aimkeypos == 7)
{
	aimkey = 0x09;//tab
}
else if (aimkeypos == 8)
{
	aimkey = 0x0c;//clear
}
else if (aimkeypos == 9)
{
	aimkey == 0x0D;//enter
}
else if (aimkeypos == 10)
{
	aimkey = 0x10;//shift
}
else if (aimkeypos == 11)
{
	aimkey = 0x11;//ctrl
}
else if (aimkeypos == 12)
{
	aimkey == 0x12;//alt
}
else if (aimkeypos == 13)
{
	aimkey == 0x14;//caps lock
}
else if (aimkeypos == 14)
{
	aimkey == 0x1B;//esc
}
else if (aimkeypos == 15)
{
	aimkey == 0x20;//space
}
else if (aimkeypos == 16)
{
	aimkey == 0x30;//0
}
else if (aimkeypos == 17)
{
	aimkey == 0x31;//1
}
else if (aimkeypos == 18)
{
	aimkey == 0x32;//2
}//dont sell this its github paste you faggot now kys
else if (aimkeypos == 19)
{
	aimkey == 0x33;//3
}
else if (aimkeypos == 20)
{
	aimkey == 0x34;//4
}
else if (aimkeypos == 21)
{
	aimkey == 0x35;//5
}
else if (aimkeypos == 22)
{
	aimkey == 0x36;//6
}
else if (aimkeypos == 23)
{
	aimkey == 0x37;//7
}
else if (aimkeypos == 24)
{
	aimkey == 0x38;//8
}
else if (aimkeypos == 25)
{
	aimkey == 0x39;//9
}
else if (aimkeypos == 26)
{
	aimkey == 0x41;//a
}
else if (aimkeypos == 27)
{
	aimkey == 0x42;//b
}
else if (aimkeypos == 28)
{
	aimkey == 0x43;//c
}
else if (aimkeypos == 29)
{
	aimkey == 0x44;//d
}
else if (aimkeypos == 30)
{
	aimkey == 0x45;//e
}
else if (aimkeypos == 31)
{
	aimkey == 0x46;//f
}
else if (aimkeypos == 32)
{
	aimkey == 0x47;//g
}
else if (aimkeypos == 33)
{
	aimkey == 0x48;//h
}
else if (aimkeypos == 34)
{
	aimkey == 0x49;//i
}
else if (aimkeypos == 35)
{
	aimkey == 0x4A;//j
}
else if (aimkeypos == 36)
{
	aimkey == 0x4B;//k
}
else if (aimkeypos == 37)
{//dont sell this its github paste you faggot now kys
	aimkey == 0x4C;//L
}
else if (aimkeypos == 38)
{
	aimkey == 0x4D;//m
}
else if (aimkeypos == 39)
{
	aimkey == 0x4E;//n
}
else if (aimkeypos == 40)
{
	aimkey == 0x4F;//o
}
else if (aimkeypos == 41)
{
	aimkey == 0x50;//p
}
else if (aimkeypos == 42)
{
	aimkey == 0x51;//q
}
else if (aimkeypos == 43)
{
	aimkey == 0x52;//r
}
else if (aimkeypos == 44)
{
	aimkey == 0x53;//s
}
else if (aimkeypos == 45)
{
	aimkey == 0x54;//t
}
else if (aimkeypos == 46)
{
	aimkey == 0x55;//u
}
else if (aimkeypos == 47)
{
	aimkey == 0x56;//v
}
else if (aimkeypos == 48)
{
	aimkey == 0x57;//w
}
else if (aimkeypos == 49)
{//dont sell this its github paste you faggot now kys
	aimkey == 0x58;//x
}
else if (aimkeypos == 50)
{
	aimkey == 0x59;//y
}
else if (aimkeypos == 51)
{
	aimkey == 0x5A;//z
}
else if (aimkeypos == 52)
{
	aimkey == 0x60;//numpad 0
}
else if (aimkeypos == 53)
{
	aimkey == 0x61;//numpad 1
}
else if (aimkeypos == 54)
{
	aimkey == 0x62;//numpad 2
}
else if (aimkeypos == 55)
{
	aimkey == 0x63;//numpad 3
}
else if (aimkeypos == 56)
{
	aimkey == 0x64;//numpad 4
}
else if (aimkeypos == 57)
{
	aimkey == 0x65;//numpad 5
}
else if (aimkeypos == 58)
{
	aimkey == 0x66;//numpad 6
}
else if (aimkeypos == 59)
{
	aimkey == 0x67;//numpad 7
}
else if (aimkeypos == 60)
{
	aimkey == 0x68;//numpad 8
}
else if (aimkeypos == 61)
{
	aimkey == 0x69;//numpad 9
}
else if (aimkeypos == 62)
{
	aimkey == 0x6A;//multiply//dont sell this its github paste you faggot now kys
}

auto entityListCopy = entityList;
float closestDistance = FLT_MAX;
DWORD_PTR closestPawn = NULL;
Uworld = KmDrv->Rpm<DWORD_PTR>(base_address + GWorld);
DWORD_PTR Gameinstance = KmDrv->Rpm<DWORD_PTR>(Uworld + OFFSETS::Gameinstance);
DWORD_PTR LocalPlayers = KmDrv->Rpm<DWORD_PTR>(Gameinstance + OFFSETS::LocalPlayers);
Localplayer = KmDrv->Rpm<DWORD_PTR>(LocalPlayers);
PlayerController = KmDrv->Rpm<DWORD_PTR>(Localplayer + OFFSETS::PlayerController);
LocalPawn = KmDrv->Rpm<DWORD_PTR>(PlayerController + OFFSETS::LocalPawn);
uintptr_t pcmc = KmDrv->Rpm<uint64_t>(PlayerController + 0x330);
PlayerState = KmDrv->Rpm<DWORD_PTR>(LocalPawn + OFFSETS::PlayerState);
DWORD_PTR PlayerCameraManager = KmDrv->Rpm<DWORD_PTR>(PlayerController + OFFSETS::Cameramanager);
PlayerCameraManager = KmDrv->Rpm<DWORD_PTR>(LocalPawn + PlayerCameraManager);
Rootcomp = KmDrv->Rpm<DWORD_PTR>(LocalPawn + OFFSETS::RootComponet);
Persistentlevel = KmDrv->Rpm<DWORD_PTR>(Uworld + OFFSETS::PersistentLevel);
uintptr_t Crrneytwep = KmDrv->Rpm<uintptr_t>(LocalPawn + 0x868);
DWORD ActorCount = KmDrv->Rpm<DWORD>(Persistentlevel + OFFSETS::ActorCount);
DWORD_PTR AOFFSETS = KmDrv->Rpm<DWORD_PTR>(Persistentlevel + OFFSETS::AActor);

DWORD_PTR GameState = KmDrv->Rpm<DWORD_PTR>(Uworld + OFFSETS::GameState);//gamestate
DWORD_PTR PlayerArray = KmDrv->Rpm<DWORD_PTR>(GameState + OFFSETS::PlayerArray);//playerarray
uint64_t CurrentVehicle = KmDrv->Rpm<uint64_t>(LocalPawn + 0x2348); //FortPlayerPawn::CurrentVehicle

bool InLobby;
InLobby = false;
if (LocalPawn) InLobby = true;
int Num = KmDrv->Rpm<int>( GameState + (OFFSETS::PlayerArray + sizeof(uintptr_t))); //reads the total number of player states in this array

for (uint32_t i = 0; i < Num; i++)
{


	//dont sell this its github paste you faggot now kys


	auto player = KmDrv->Rpm<uintptr_t>(PlayerArray + i * OFFSETS::CurrentActor);
	auto CurrentActor = KmDrv->Rpm<uintptr_t>(player + OFFSETS::Private);//PawnPrivate

	if (!CurrentActor) {
		continue;
	}

	//dont sell this its github paste you faggot now kys
	
	int NewPlayerLocationX;
	int NewPlayerLocationY;

	//uint64_t CurrentActor = read<uint64_t>(AOFFSETS + i * OFFSETS::CurrentActor);
   // if (read<float>(CurrentActor + OFFSETS::Revivefromdbnotime) != 10) continue;
	uint64_t CurrentActorMesh = KmDrv->Rpm<uint64_t>(CurrentActor + OFFSETS::Mesh);
	int MyTeamId = KmDrv->Rpm<int>(PlayerState + OFFSETS::TeamId);
	DWORD64 otherPlayerState = KmDrv->Rpm<uint64_t>(CurrentActor + 0x290);
	int ActorTeamId = KmDrv->Rpm<int>(otherPlayerState + OFFSETS::TeamId);
	auto isDBNO = (KmDrv->Rpm<char>(CurrentActor + 0x7c2) >> 4) & 1;

	auto entityListCopy = entityList;
	if (MyTeamId == ActorTeamId) continue;
	if (LocalPawn)//ik worst way to do lobby esp but i really dont give a fuck on this paste common sense - bizzy
	{
		VisDist;

	}
	else
	{
		VisDist = 2400;

	}
	if (slefESP)
	{
		continue;//dont sell this its github paste you faggot now kys
	}
	else {
		if (CurrentActor == LocalPawn) continue;
	}
	if (CurrentActor == LocalPawn) continue;
	Vector3 Headpos = GetBoneWithRotation(CurrentActorMesh, 67);
	Vector3 head2 = GetBoneWithRotation(CurrentActorMesh, 68);
	Vector3 footpos = GetBoneWithRotation(CurrentActorMesh, 0);
	localactorpos = KmDrv->Rpm<Vector3>(Rootcomp + OFFSETS::LocalActorPos);
	auto normal_head = ProjectWorldToScreen(Headpos);
	float distance = localactorpos.Distance(Headpos) / ChangerFOV;

	Vector3 bone15 = GetBoneWithRotation(CurrentActorMesh, 50);
	Vector3 top = ProjectWorldToScreen(bone15);

	Vector3 bone0 = GetBoneWithRotation(CurrentActorMesh, 0);
	Vector3 bottom = ProjectWorldToScreen(bone0);
	Vector3 Headbox = ProjectWorldToScreen(Vector3(Headpos.x, Headpos.y, Headpos.z + 15));
	Vector3 HeadElvar = ProjectWorldToScreen(Vector3(Headpos.x, Headpos.y, Headpos.z));
	Vector3 FeetElvar = ProjectWorldToScreen(Vector3(footpos.x, footpos.y, footpos.z));
	//Vector3 Toebox = ProjectWorldToScreen(Vector3(Toepos.x, Toepos.y, Toepos.z + 15));
	Vector3 w2shead = ProjectWorldToScreen(Headpos);

	float BoxHeight = (float)(Headbox.y - bottom.y);
	float BoxWidth = BoxHeight * 0.350f;

	float LeftX = (float)Headbox.x - (BoxWidth / 1);
	float LeftY = (float)bottom.y;

	int center_x = GetSystemMetrics(0) / 2 - 3;
	int center_y = GetSystemMetrics(1) / 2 - 3;



	//dont sell this its github paste you faggot now kys

	int dynamedfov = AimFOV - ((fov - 80) * 2.5);
	float CornerHeight = abs(Headbox.y - bottom.y);
	float CornerWidth = CornerHeight * BoxWidthValue;
	if (distance < VisDist)
	{
		
		if (Esp)
		{ 

			if (BOXESP)
			{
				//Normalbox(Headbox.x - (CornerWidth / 2), Headbox.y, CornerWidth, CornerHeight, IM_COL32(255, 255, 0, 255), 1.5, 0);


				if (IsVisible(CurrentActorMesh)) {
					Normalbox(Headbox.x - (CornerWidth / 2), Headbox.y, CornerWidth, CornerHeight, ImColor({ vischick[0], vischick[1], vischick[2] }), 1.5, 0); // Green color (0, 255, 0)
				}
				if (!IsVisible(CurrentActorMesh)) {
					Normalbox(Headbox.x - (CornerWidth / 2), Headbox.y, CornerWidth, CornerHeight, ImColor({ notvischick[0], notvischick[1], notvischick[2] }), 1.5, 0);
				}

				
				//if (rainbowesp)
				//{
				//	DrawBox(Headbox.x - (CornerWidth / 2), Headbox.y, CornerWidth, CornerHeight, RGB, 2.5f);
				//}
				//else {
				//	/*rgba(0, 255, 231, 1)*/
				//	DrawBox(Headbox.x - (CornerWidth / 2), Headbox.y, CornerWidth, CornerHeight, ImColor({ boxchick[0], boxchick[1], boxchick[2] }), 3.0f);
				//}

			}//dont sell this its github paste you faggot now kys


			if (VisibleCheck) {
				float LastSumbitTime = KmDrv->Rpm<float>(OFFSETS::Mesh + 0x360);
				float LastRenderTimeOnScreen = KmDrv->Rpm<float>(OFFSETS::Mesh + 0x368);

				bool Visible = LastRenderTimeOnScreen + 0.06f >= LastSumbitTime;

			}

			//dont sell this its github paste you faggot now kys

			//dont sell this its github paste you faggot now kys
			if (Skeleton)
			{
				skeleton(CurrentActorMesh, ImColor({ 0, 255, 255 }));
			}

			



			if (WindowStreamProof) {
				SetWindowDisplayAffinity(Window, WDA_EXCLUDEFROMCAPTURE);
			}


			if (fpsCounter)
			{
				char dist[64];
				XorS(frame, "%.1f Fps |  Overlay FPS  \n");
				sprintf_s(dist, frame.decrypt(), 1.0f / ImGui::GetIO().DeltaTime);
				DrawText1(900, 15, dist, &Col.red);
			}
			if (Esp_Distance)
			{
				char name[64];
				sprintf_s(name, ("%2.fm"), distance);
				DrawString(16, bottom.x, bottom.y - 5, &Col.white, true, true, name);
			}

			if (Esp_line)
			{
				
				if (IsVisible(CurrentActorMesh)) {
					DrawLine(Width / 2, Height, bottom.x, bottom.y, &Col.red, 1.5);
				}
				if (!IsVisible(CurrentActorMesh)) {
					DrawLine(Width / 2, Height, bottom.x, bottom.y, &Col.white, 1.5);
				}

				//DrawLine(Width / 2, Height, bottom.x, bottom.y, &Col.white, 1.5);
				
			}

			

		}//dont sell this its github paste you faggot now kys
	}
	auto dx = w2shead.x - (Width / 2);
	auto dy = w2shead.y - (Height / 2);
	auto dist = sqrtf(dx * dx + dy * dy);


	if (IsVisible(CurrentActorMesh)) {

		if (dist < AimFOV && dist < closestDistance) {
			closestDistance = dist;
			closestPawn = CurrentActor;

		}
	}
}

if (Aimbot)
{
		//hotkeys::aimkey
		if (Aimbot && closestPawn && GetAsyncKeyState(VK_SHIFT) < 0) {
			AimAt(closestPawn);
		}
	//dont sell this its github paste you faggot now kys

}
}


void render() {
	ImGui_ImplDX9_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	static int maintabs = 0;
	ImDrawList* draw;

	
	//Functions for the demo
	static bool boxesp = false;
	static bool thirddimensionbox = false;
	static bool corneredbox = false;
	static bool fillbox = false;
	static bool skeleton = false;
	static float thickness = 0.f;
	static char inputtext[64] = "";
	float boxcolor[4] = { 1.00f, 0.79f, 0.05f, 1.00f };
	float fillcolor[4] = { 1.00f, 0.79f, 0.05f, 1.00f };
	static int playermodel = 0;

	if (GetAsyncKeyState(0x2D) & 1)
	{
		ShowMenu = !ShowMenu;
	}

	if (ShowMenu)
	{
		
		ImGui::Begin("PasterX Ud", NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize);
		

			
			ImGui::SetWindowSize(ImVec2(300, 500));


			ImGui::Checkbox("Aimbot", &Aimbot);
			ImGui::Checkbox("Show Fov", &square_fov);
			ImGui::SliderFloat("Fov Size", &AimFOV, 1, 250);
			ImGui::SliderFloat("Smoothing ", &smooth, 1, 10);
			ImGui::Spacing();
			ImGui::Checkbox("Box", &BOXESP);
			ImGui::Checkbox("Snaplines", &Esp_line);
			ImGui::Checkbox("Distance", &Esp_Distance);
			ImGui::Checkbox("Skeleton", &Skeleton);




			ImGui::End();

		

	}//dont sell this its github paste you faggot now kys




	DrawESP();

	ImGui::EndFrame();
	D3dDevice->SetRenderState(D3DRS_ZENABLE, false);
	D3dDevice->SetRenderState(D3DRS_ALPHABLENDENABLE, false);
	D3dDevice->SetRenderState(D3DRS_SCISSORTESTENABLE, false);
	D3dDevice->Clear(0, NULL, D3DCLEAR_TARGET, D3DCOLOR_ARGB(0, 0, 0, 0), 1.0f, 0);

	if (D3dDevice->BeginScene() >= 0)
	{
		ImGui::Render();
		ImGui_ImplDX9_RenderDrawData(ImGui::GetDrawData());
		D3dDevice->EndScene();
	}
	HRESULT result = D3dDevice->Present(NULL, NULL, NULL, NULL);

	if (result == D3DERR_DEVICELOST && D3dDevice->TestCooperativeLevel() == D3DERR_DEVICENOTRESET)
	{
		ImGui_ImplDX9_InvalidateDeviceObjects();
		D3dDevice->Reset(&d3dpp);
		ImGui_ImplDX9_CreateDeviceObjects();
	}
}
//dont sell this its github paste you faggot now kys
MSG Message = { NULL };
int Loop = 0;
void xMainLoop()
{
	static RECT old_rc;
	ZeroMemory(&Message, sizeof(MSG));

	while (Message.message != WM_QUIT)
	{
		if (PeekMessage(&Message, Window, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&Message);
			DispatchMessage(&Message);
		}

		HWND hwnd_active = GetForegroundWindow();

		if (hwnd_active == hwnd) {
			HWND hwndtest = GetWindow(hwnd_active, GW_HWNDPREV);
			SetWindowPos(Window, hwndtest, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE);
		}

		if (GetAsyncKeyState(0x23) & 1)
			exit(8);

		RECT rc;
		POINT xy;

		ZeroMemory(&rc, sizeof(RECT));
		ZeroMemory(&xy, sizeof(POINT));
		GetClientRect(hwnd, &rc);
		ClientToScreen(hwnd, &xy);
		rc.left = xy.x;
		rc.top = xy.y;

		ImGuiIO& io = ImGui::GetIO();
		io.IniFilename = NULL;
		io.ImeWindowHandle = hwnd;
		io.DeltaTime = 1.0f / 60.0f;
		//dont sell this its github paste you faggot now kys
		POINT p;
		GetCursorPos(&p);
		io.MousePos.x = p.x - xy.x;
		io.MousePos.y = p.y - xy.y;

		if (GetAsyncKeyState(VK_LBUTTON)) {
			io.MouseDown[0] = true;
			io.MouseClicked[0] = true;
			io.MouseClickedPos[0].x = io.MousePos.x;
			io.MouseClickedPos[0].x = io.MousePos.y;
		}
		else
			io.MouseDown[0] = false;

		if (rc.left != old_rc.left || rc.right != old_rc.right || rc.top != old_rc.top || rc.bottom != old_rc.bottom)
		{
			old_rc = rc;

			Width = rc.right;
			Height = rc.bottom;

			d3dpp.BackBufferWidth = Width;
			d3dpp.BackBufferHeight = Height;
			SetWindowPos(Window, (HWND)0, xy.x, xy.y, Width, Height, SWP_NOREDRAW);
			D3dDevice->Reset(&d3dpp);
		}
		render();
		if (Loop == 0) {
			XorS(base, "Process base address: %p.\n");
			//printf(base.decrypt(), (void*)base_address);
		}
		Loop = Loop + 1;
	}
	ImGui_ImplDX9_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImGui::DestroyContext();

	DestroyWindow(Window);
}
//dont sell this its github paste you faggot now kys
LRESULT CALLBACK WinProc(HWND hWnd, UINT Message, WPARAM wParam, LPARAM lParam)
{
	if (ImGui_ImplWin32_WndProcHandler(hWnd, Message, wParam, lParam))
		return true;

	switch (Message)
	{
	case WM_DESTROY:
		xShutdown();
		PostQuitMessage(0);
		exit(4);
		break;
	case WM_SIZE:
		if (D3dDevice != NULL && wParam != SIZE_MINIMIZED)
		{
			ImGui_ImplDX9_InvalidateDeviceObjects();
			d3dpp.BackBufferWidth = LOWORD(lParam);
			d3dpp.BackBufferHeight = HIWORD(lParam);
			HRESULT hr = D3dDevice->Reset(&d3dpp);
			if (hr == D3DERR_INVALIDCALL)
				IM_ASSERT(0);
			ImGui_ImplDX9_CreateDeviceObjects();
		}
		break;
	default:
		return DefWindowProc(hWnd, Message, wParam, lParam);
		break;
	}
	return 0;
}
//dont sell this its github paste you faggot now kys
void xShutdown()
{
	TriBuf->Release();
	D3dDevice->Release();
	p_Object->Release();

	DestroyWindow(Window);
	UnregisterClass("Bluestacks", NULL);
}

//dont sell this its github paste you faggot now kys//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
//dont sell this its github paste you faggot now kys
