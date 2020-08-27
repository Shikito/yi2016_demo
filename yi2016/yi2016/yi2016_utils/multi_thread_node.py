import time
import threading

from rclpy.node import Node

class MultiThreadNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.__threads: List[Thread] = []
    
    @property
    # def threads(self) -> Iterator[Thread]:
    def threads(self):
        """Get threads that have been created on this node."""
        yield from self.__threads

    def create_thread(
        self,
        timer_period_sec: float,
        # callback: Callable,
        callback,
    # ) -> Thread:
    ):
        """
        Create a new Thread.
        """
        def thread_loop(timer_period_sec):
            while True:
                callback()
                time.sleep(timer_period_sec)
        
        thread = threading.Thread(target=thread_loop, daemon=True, args=(timer_period_sec,))
        self.__threads.append(thread)
        return thread

    # def destroy_thread(self, thread: Thread) -> bool:
    def destroy_thread(self, thread) -> bool:
        """
        Destroy a thread created by the node.

        :return: ``True`` if successful, ``False``otherwise.
        """
        if thread in self.__threads:
            self.__threads.remove(thread)
            try:
                thread.join()
            except RuntimeError: # <= joinのエラー処理ってこれでいいのかわからない(笑)
                return False
            return True
        return False

    def destroy_node(self) -> bool:
        while self.__threads:
            self.destroy_thread(self.__threads[0])
        super().destroy_node()

    def request_service_sync(self, client, req):
        future = client.call_async(req)

        while not future.done():
            self.get_logger().info('waiting for response')
            time.sleep(0.01)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))

        return response
