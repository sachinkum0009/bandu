from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional, Literal, Union
from datetime import datetime


class ContentItem(BaseModel):
    """Represents individual content items within a message"""
    type: str
    text: str


class UsageMetadata(BaseModel):
    """Usage statistics for AI model calls"""
    input_tokens: int
    output_tokens: int
    total_tokens: int


class OllamaResponseMetadata(BaseModel):
    """Metadata from Ollama AI model responses"""
    model: str
    created_at: str
    done: bool
    done_reason: str
    total_duration: int
    load_duration: int
    prompt_eval_count: int
    prompt_eval_duration: int
    eval_count: int
    eval_duration: int
    model_name: str


class OpenAIResponseMetadata(BaseModel):
    """Metadata from OpenAI model responses"""
    finish_reason: str
    model_name: str
    system_fingerprint: Optional[str] = None
    service_tier: Optional[str] = None


class ResponseMetadata(BaseModel):
    """Flexible metadata that can handle both Ollama and OpenAI formats"""
    # OpenAI fields
    finish_reason: Optional[str] = None
    model_name: Optional[str] = None
    system_fingerprint: Optional[str] = None
    service_tier: Optional[str] = None
    
    # Ollama fields
    model: Optional[str] = None
    created_at: Optional[str] = None
    done: Optional[bool] = None
    done_reason: Optional[str] = None
    total_duration: Optional[int] = None
    load_duration: Optional[int] = None
    prompt_eval_count: Optional[int] = None
    prompt_eval_duration: Optional[int] = None
    eval_count: Optional[int] = None
    eval_duration: Optional[int] = None


class ToolCall(BaseModel):
    """Represents a tool call made by an AI agent"""
    name: str
    args: Dict[str, Any]
    id: str
    type: str


class SystemMultimodalMessage(BaseModel):
    """System message with multimodal content support"""
    content: List[ContentItem]
    additional_kwargs: Dict[str, Any] = Field(default_factory=dict)
    response_metadata: Dict[str, Any] = Field(default_factory=dict)
    id: str
    images: List[Any] = Field(default_factory=list)


class HumanMessage(BaseModel):
    """Message from human user"""
    content: str
    additional_kwargs: Dict[str, Any] = Field(default_factory=dict)
    response_metadata: Dict[str, Any] = Field(default_factory=dict)
    id: str


class AIMessage(BaseModel):
    """Message from AI agent"""
    content: str
    additional_kwargs: Dict[str, Any] = Field(default_factory=dict)
    response_metadata: Dict[str, Any] = Field(default_factory=dict)  # Changed to flexible dict
    id: str
    tool_calls: Optional[List[ToolCall]] = None
    usage_metadata: Optional[UsageMetadata] = None


class ToolMessage(BaseModel):
    """Message representing tool execution result"""
    content: str
    additional_kwargs: Dict[str, Any] = Field(default_factory=dict)
    response_metadata: Dict[str, Any] = Field(default_factory=dict)
    id: str
    name: str
    tool_call_id: str


class AgentConversation(BaseModel):
    """Represents a conversation for a single agent"""
    messages: List[Union[SystemMultimodalMessage, HumanMessage, AIMessage, ToolMessage]]


class ChunkData(BaseModel):
    """Root model representing the complete chunk data structure"""
    data: Dict[str, AgentConversation]

    def get_last_message(self) -> str:
        """Get the content of the last message from the first agent in the data"""
        if not self.data:
            return ""
        first_agent = next(iter(self.data.values()))
        if not first_agent.messages:
            return ""
        
        last_message = first_agent.messages[-1]
        if hasattr(last_message, 'content'):
            content = last_message.content
            # Handle SystemMultimodalMessage content (list of ContentItem)
            if isinstance(content, list):
                return " ".join([item.text for item in content if hasattr(item, 'text')])
            return content
        return ""
    
    def get_last_ai_message(self) -> str:
        """Get the content of the last AI message with actual content from the first agent"""
        if not self.data:
            return ""
        first_agent = next(iter(self.data.values()))
        if not first_agent.messages:
            return ""
        
        # Find all AI messages with content
        ai_messages_with_content = [
            msg for msg in first_agent.messages 
            if isinstance(msg, AIMessage) and msg.content.strip()
        ]
        
        if ai_messages_with_content:
            return ai_messages_with_content[-1].content
        return ""
    
    @classmethod
    def from_dict(cls, raw_data: Dict[str, Any]) -> "ChunkData":
        """Create ChunkData from the raw dictionary format"""
        agents = {}
        
        for agent_id, agent_data in raw_data.items():
            messages = []
            
            for msg_data in agent_data["messages"]:
                # Handle LangChain message objects directly
                if hasattr(msg_data, '__class__') and hasattr(msg_data, 'content'):
                    # This is a LangChain message object
                    if msg_data.__class__.__name__ == "SystemMultimodalMessage":
                        messages.append(SystemMultimodalMessage(
                            content=msg_data.content,
                            additional_kwargs=getattr(msg_data, 'additional_kwargs', {}),
                            response_metadata=getattr(msg_data, 'response_metadata', {}),
                            id=getattr(msg_data, 'id', ''),
                            images=getattr(msg_data, 'images', [])
                        ))
                    elif msg_data.__class__.__name__ == "HumanMessage":
                        messages.append(HumanMessage(
                            content=msg_data.content,
                            additional_kwargs=getattr(msg_data, 'additional_kwargs', {}),
                            response_metadata=getattr(msg_data, 'response_metadata', {}),
                            id=getattr(msg_data, 'id', '')
                        ))
                    elif msg_data.__class__.__name__ == "AIMessage":
                        messages.append(AIMessage(
                            content=msg_data.content,
                            additional_kwargs=getattr(msg_data, 'additional_kwargs', {}),
                            response_metadata=getattr(msg_data, 'response_metadata', {}),
                            id=getattr(msg_data, 'id', ''),
                            tool_calls=getattr(msg_data, 'tool_calls', None),
                            usage_metadata=getattr(msg_data, 'usage_metadata', None)
                        ))
                    elif msg_data.__class__.__name__ == "ToolMessage":
                        messages.append(ToolMessage(
                            content=msg_data.content,
                            additional_kwargs=getattr(msg_data, 'additional_kwargs', {}),
                            response_metadata=getattr(msg_data, 'response_metadata', {}),
                            id=getattr(msg_data, 'id', ''),
                            name=getattr(msg_data, 'name', ''),
                            tool_call_id=getattr(msg_data, 'tool_call_id', '')
                        ))
                else:
                    # Handle dictionary format (fallback)
                    msg_dict = msg_data if isinstance(msg_data, dict) else msg_data.__dict__
                    
                    # Logic to determine message type
                    if isinstance(msg_dict.get("content"), list):
                        messages.append(SystemMultimodalMessage(**msg_dict))
                    elif "tool_call_id" in msg_dict:
                        messages.append(ToolMessage(**msg_dict))
                    elif ("tool_calls" in msg_dict or 
                          "usage_metadata" in msg_dict or 
                          msg_dict.get("response_metadata", {}).get("finish_reason") or
                          msg_dict.get("response_metadata", {}).get("model_name")):
                        messages.append(AIMessage(**msg_dict))
                    else:
                        messages.append(HumanMessage(**msg_dict))
            
            agents[agent_id] = AgentConversation(messages=messages)
        
        return cls(data=agents)


# Example usage and validation
if __name__ == "__main__":
    # Sample data based on your chunk structure
    sample_chunk_data = {
        'agent1': {
            'messages': [
                {
                    'content': [{'type': 'text', 'text': '<description>\nYou are an AI agent deployed on Robot. You have access to the tools which can be used to control the robot e.g. GetRobotTemperatureTool.\n</description>\n<rules>\nAlways follow the instructions.\n</rules>\n<capabilities>\nGet the temperature of the robot using tool\n</capabilities>\n<behaviors>\nFriendly robot, gives polite answers\nAlways ask for clarification if the task is not clear.\n</behaviors>\n'}],
                    'additional_kwargs': {},
                    'response_metadata': {},
                    'id': '0c0f886d-b823-4657-9e1e-80f5f087ede2',
                    'images': []
                },
                {
                    'content': 'Can you help me understand the current temperature of the robot?',
                    'additional_kwargs': {},
                    'response_metadata': {},
                    'id': '8bd74f5d-8180-405b-9fbc-2b047b5b8d8d'
                },
                {
                    'content': '',
                    'additional_kwargs': {'tool_calls': [{'index': 0, 'id': 'call_494dJxrnvoBUskZ3pxad65vt', 'function': {'arguments': '{}', 'name': 'get_robot_temperature'}, 'type': 'function'}]},
                    'response_metadata': {
                        'finish_reason': 'tool_calls',
                        'model_name': 'gpt-4o-2024-08-06',
                        'system_fingerprint': 'fp_cbf1785567',
                        'service_tier': 'default'
                    },
                    'id': 'run--9498acb4-2063-4218-beb0-bdb9a9c7e6a1-0',
                    'tool_calls': [
                        {
                            'name': 'get_robot_temperature',
                            'args': {},
                            'id': 'call_494dJxrnvoBUskZ3pxad65vt',
                            'type': 'tool_call'
                        }
                    ]
                },
                {
                    'content': 'The robot temperature is 55°C',
                    'name': 'get_robot_temperature',
                    'id': '174e608d-8f01-45af-8e29-dbc9a9324f4a',
                    'tool_call_id': 'call_494dJxrnvoBUskZ3pxad65vt',
                    'additional_kwargs': {},
                    'response_metadata': {}
                },
                {
                    'content': 'The current temperature of the robot is 55°C. If you have any concerns about this temperature or need further assistance, please let me know!',
                    'additional_kwargs': {},
                    'response_metadata': {
                        'finish_reason': 'stop',
                        'model_name': 'gpt-4o-2024-08-06',
                        'system_fingerprint': 'fp_cbf1785567',
                        'service_tier': 'default'
                    },
                    'id': 'run--b6ee2631-f6cd-4758-b3c0-50723d3568b4-0',
                    'tool_calls': None,
                    'usage_metadata': None
                }
            ]
        }
    }
    
    # Create and validate the model
    try:
        chunk = ChunkData.from_dict(sample_chunk_data)
        print("✅ Model validation successful!")
        print(f"Number of agents: {len(chunk.data)}")
        print(f"Agent1 has {len(chunk.data['agent1'].messages)} messages")
        
        # Print message types
        for i, msg in enumerate(chunk.data['agent1'].messages):
            print(f"Message {i}: {type(msg).__name__}")
        
        # Test the new methods
        print(f"\n📨 Last Message (any type): '{chunk.get_last_message()}'")
        print(f"\n🤖 Last AI Message with Content: '{chunk.get_last_ai_message()}'")
        
        # Find and print all AI messages for comparison
        ai_messages = [msg for msg in chunk.data['agent1'].messages if isinstance(msg, AIMessage)]
        if ai_messages:
            print(f"\n🔍 Found {len(ai_messages)} AI messages:")
            for i, ai_msg in enumerate(ai_messages):
                print(f"\nAI Message {i+1}:")
                print(f"Content: '{ai_msg.content}'")
                print(f"ID: {ai_msg.id}")
                if ai_msg.tool_calls:
                    print(f"Tool calls: {len(ai_msg.tool_calls)} - {[tc.name for tc in ai_msg.tool_calls]}")
                if ai_msg.usage_metadata:
                    print(f"Token usage: {ai_msg.usage_metadata.total_tokens} total tokens")
        else:
            print("\n❌ No AI messages found")
            
    except Exception as e:
        print(f"❌ Validation failed: {e}")