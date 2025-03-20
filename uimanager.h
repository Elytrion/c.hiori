#pragma once
#include "cprocessing.h"

class DebugGraphics;

enum class UIEventType
{
	OnMouseDown,
	OnMouseTriggered,
	OnMouseUp,
	OnKeyDown,
	OnKeyTriggered,
	OnKeyUp,
	OnHover,
	NONE
};

// Alias for a generic callback
using EventCallback = std::function<void()>;

struct UIEventTrigger
{
	UIEventType type = UIEventType::NONE;
    CP_KEY key;     // which key has to be triggered 
    CP_MOUSE mouse; // which mouse has to be triggered
    EventCallback callback;
};

struct UIComponent
{
	float position[2];   // Top-left position of the element
	float size[2];       // Size of the element
	std::string str;	 // optional string for text on the button
	std::vector<UIEventTrigger> events; // List of event triggers for this component
	int drawIndex{-1};
	int textDrawIndex{-1};

    bool Contains(float x, float y)
    {
        return x >= position[0] && x <= position[0] + size[0] &&
               y >= position[1] && y <= position[1] + size[1];
    }
};

struct UIComponentConfig
{
	float x, y;			// pos
	float w, h;			// size
	float r, g, b, a;	// color
	std::string str;	// optional string for text on the button
	float sx{ 0.0f }, sy{ 0.0f };			// string pos offset from pos
	float sr {0.9f}, sg{ 0.9f }, sb{ 0.9f }, sa{ 1.0f };	// string color
	float textSize{ 14.0f };
};

class UIManager
{
private:
	DebugGraphics* drawer;
    std::vector<UIComponent> uiComponents;
	
public:
	UIManager(DebugGraphics* inDrawer) : drawer{ inDrawer } {}

    void Update();

	int AddRectUIButton(UIComponentConfig config, std::vector<UIEventTrigger> events);
	int AddCircleUIButton(UIComponentConfig config, std::vector<UIEventTrigger> events);

	void AddEventToUIComponent(int index, UIEventTrigger event);
	void ChangeConfig(int index, UIComponentConfig config);

	void ClearUI();
};