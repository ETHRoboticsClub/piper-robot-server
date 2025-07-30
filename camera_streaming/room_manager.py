from livekit import api 

class RoomManager:
    def __init__(self):
        self.lkapi = api.LiveKitAPI()
    
    async def create_room(self, name) -> None:
        await self.lkapi.room.create_room(
            name=name,
            empty_timeout = 10 * 60,
            max_participants=2,
        )

    async def delete_room(self, name) -> None:
        await self.lkapi.room.delete_room(
            name=name
        )
        
    async def list_rooms(self) -> list[api.Room]:
        return await self.lkapi.room.list_rooms()
        
        
