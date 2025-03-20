#include "pch.h"
#include "chioriMath.h"
#include "uimanager.h"
#include "graphics.h"


int UIManager::AddRectUIButton(UIComponentConfig config, std::vector<UIEventTrigger> events)
{
    UIComponent button;
    button.position[0] = config.x;
    button.position[1] = config.y;
    chiori::cVec2 pos = { config.x,config.y };
    button.size[0] = config.w;
    button.size[1] = config.h;
    chiori::cVec2 size = { config.w, config.h };
    

    for (const UIEventTrigger& event : events)
    {
        if (event.type != UIEventType::NONE)
        {
            button.events.push_back(event);
        }
    }
    
    button.drawIndex = drawer->AddUIRect(pos, size, { config.r, config.g, config.b, config.a });

    if (config.str.size() > 0)
    {
        pos += { config.sx, config.sy };
        button.textDrawIndex = drawer->AddUIText(pos, config.str, config.textSize, { config.sr, config.sg, config.sb, config.sa });
    }
    
    uiComponents.push_back(button);
    return uiComponents.size() - 1;
}

int UIManager::AddCircleUIButton(UIComponentConfig config, std::vector<UIEventTrigger> events)
{
    UIComponent button;
    button.position[0] = config.x;
    button.position[1] = config.y;
    chiori::cVec2 center = { config.x,config.y };
    button.size[0] = config.w;
    button.size[1] = config.w;

    for (const UIEventTrigger& event : events)
    {
        if (event.type != UIEventType::NONE)
        {
            button.events.push_back(event);
        }
    }
    
    button.drawIndex = drawer->AddUICircle(center, config.w, { config.r, config.g, config.b, config.a });
    if (config.str.size() > 0)
    {
        center += { config.sx,config.sy };
        button.textDrawIndex = drawer->AddUIText(center, config.str, config.textSize, { config.sr, config.sg, config.sb, config.sa });
    }

    uiComponents.push_back(button);

    return uiComponents.size() - 1;
}

void UIManager::AddEventToUIComponent(int index, UIEventTrigger event)
{
    auto& component = uiComponents[index];
    component.events.push_back(event);
}

void UIManager::ChangeConfig(int index, UIComponentConfig config)
{
    auto& button = uiComponents[index];
    auto& uiElements = drawer->getUIElements();

    button.position[0] = config.x;
    button.position[1] = config.y;
    button.size[0] = config.w;
    button.size[1] = config.h;
    button.str = config.str;

    auto& bgUIElement = uiElements[button.drawIndex];
    bgUIElement.position[0] = config.x;
    bgUIElement.position[1] = config.y;
    bgUIElement.size[0] = config.w;
    bgUIElement.size[1] = config.h;
    bgUIElement.color = drawer->ConvertColor({config.r, config.g, config.b, config.a });

    if (config.str.size() > 0)
    {
        auto& textUIElement = uiElements[button.textDrawIndex];
    }
    
    
}

void UIManager::Update()
{
    float mouseX = CP_Input_GetMouseX();
    float mouseY = CP_Input_GetMouseY();
    
    // Process each UI component
    for (auto& component : uiComponents)
    {
        bool isHovered = component.Contains(mouseX, mouseY);
        for (const auto& trigger : component.events)
        {
            switch (trigger.type)
            {
            case UIEventType::OnHover:
                if (isHovered)
                    trigger.callback();
                break;
            case UIEventType::OnMouseDown:
                if (isHovered && CP_Input_MouseDown(trigger.mouse))
                    trigger.callback(); 
                break;
            case UIEventType::OnMouseTriggered:
                if (isHovered && CP_Input_MouseTriggered(trigger.mouse))
                    trigger.callback(); 
                break;
            case UIEventType::OnMouseUp:
                if (isHovered && CP_Input_MouseReleased(trigger.mouse))
                    trigger.callback(); 
                break;
            case UIEventType::OnKeyDown:
                if (isHovered && CP_Input_KeyDown(trigger.key))
                    trigger.callback();
                break;
            case UIEventType::OnKeyTriggered:
                if (isHovered && CP_Input_KeyTriggered(trigger.key))
                    trigger.callback();
                break;
            case UIEventType::OnKeyUp:
                if (isHovered && CP_Input_KeyReleased(trigger.key))
                    trigger.callback();
                break;
            default:
                break;
            }
        }
    }
}

void UIManager::ClearUI()
{
    uiComponents.clear();
}