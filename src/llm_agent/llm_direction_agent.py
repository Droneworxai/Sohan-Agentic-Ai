#!/usr/bin/env python3
"""
AGENTIC LLM: Analyzes conditions and makes intelligent decisions
"""
import json
from groq import Groq
from datetime import datetime

GROQ_API_KEY = "YOUR_GROQ_API_KEY_HERE"

def get_llm_decision():
    """LLM analyzes conditions and decides direction"""
    
    client = Groq(api_key=GROQ_API_KEY)
    
    # Get current context
    current_hour = datetime.now().hour
    time_of_day = "morning" if 6 <= current_hour < 12 else "afternoon" if 12 <= current_hour < 18 else "evening"
    
    prompt = f"""You are an autonomous agricultural robotics agent making real-time treatment decisions.

CURRENT SITUATION:
- Location: Lavender farm, 26-waypoint rectangular treatment loop
- Time: {time_of_day} ({current_hour}:00 hours)
- Battery: 85%
- Wind: Light breeze from west (3 m/s)
- Temperature: 18°C
- Field moisture: Moderate
- Last treatment: 3 days ago

YOUR TASK:
Decide whether to treat the loop CLOCKWISE or COUNTER-CLOCKWISE.

FACTORS TO CONSIDER:
- Sun position affects camera weed detection accuracy
- Wind direction impacts chemical spray drift
- Battery life vs treatment time
- Plant stress is highest on sun-facing rows
- Morning dew affects chemical adhesion
- Afternoon heat stresses plants differently

THINK THROUGH:
1. What are the key tradeoffs?
2. Which direction maximizes treatment effectiveness?
3. What could go wrong with each choice?
4. What's the optimal decision given ALL factors?

Respond with ONLY valid JSON:
{{
  "direction": "clockwise" or "counter-clockwise",
  "reasoning": "2-3 sentences explaining your decision with specific factors",
  "confidence": "high/medium/low",
  "tradeoffs_considered": ["factor1", "factor2", "factor3"]
}}

Think carefully - this affects real crop treatment! NO markdown, ONLY JSON.
"""
    
    print("\nLLM AGENT ANALYZING FIELD CONDITIONS...")
    print("=" * 70)
    
    response = client.chat.completions.create(
        model="llama-3.1-8b-instant",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.7,  # Higher temp for real reasoning
        max_tokens=500
    )
    
    response_text = response.choices[0].message.content.strip()
    response_text = response_text.replace('```json', '').replace('```', '').strip()
    
    try:
        decision = json.loads(response_text)
        
        print("✅ LLM DECISION:")
        print(f"  Direction: {decision['direction'].upper()}")
        print(f"  Confidence: {decision['confidence']}")
        print()
        print("🧠 REASONING:")
        print(f"  {decision['reasoning']}")
        print()
        print("⚖️  TRADEOFFS CONSIDERED:")
        for factor in decision.get('tradeoffs_considered', []):
            print(f"    • {factor}")
        print("=" * 70)
        
        return decision
        
    except json.JSONDecodeError as e:
        print(f" Error parsing LLM response: {e}")
        print(f"Raw response: {response_text}")
        return {
            "direction": "clockwise",
            "reasoning": "Fallback to default pattern",
            "confidence": "low"
        }

def apply_decision(waypoints, direction):
    """Apply LLM decision to waypoint order"""
    if direction.lower() == "counter-clockwise":
        print("\n Reversing waypoints for COUNTER-CLOCKWISE navigation")
        return waypoints[::-1]
    else:
        print("\n Using CLOCKWISE waypoint sequence")
        return waypoints

if __name__ == '__main__':
    print("\n" + "=" * 70)
    print("AUTONOMOUS AGRICULTURAL LLM AGENT")
    print("=" * 70)
    
    # Load mission
    with open('mission_plan.json', 'r') as f:
        mission = json.load(f)
    
    # LLM makes decision
    decision = get_llm_decision()
    
    # Apply decision
    final_waypoints = apply_decision(mission['local_coords'], decision['direction'])
    
    # Update mission with LLM metadata
    mission['strategy'] = f"LLM-optimized {decision['direction']} treatment"
    mission['llm_reasoning'] = decision['reasoning']
    mission['llm_confidence'] = decision.get('confidence', 'unknown')
    mission['llm_factors'] = decision.get('tradeoffs_considered', [])
    mission['local_coords'] = final_waypoints
    
    with open('mission_plan.json', 'w') as f:
        json.dump(mission, f, indent=2)
    
    print(f"\n MISSION UPDATED WITH LLM DECISION")
    print(f"  Waypoints: {len(final_waypoints)}")
    print(f"  Strategy: {mission['strategy']}")
    print("=" * 70)