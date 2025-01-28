from multiprocessing import Process, Pipe
class FastPool:
    def __init__(self,process_count):
        self.process_count = process_count
        self.pipes = []
        self.processes = []
        shutdown_pipe, shutdown_query_pipe = Pipe()
        self.shutdown_pipe = shutdown_pipe
        # 1 = ready to recieve input
        # 0 = not ready / processing input
        self.process_status = []
        for idx in range(process_count):
            parent_connection, child_connection = Pipe()
            self.pipes.append([parent_connection,child_connection])
            process = Process(target=self.process_handler, args=(child_connection,shutdown_query_pipe))
            process.start()
            self.process_status.append(1)

    @staticmethod
    def process_handler(communication_pipe,shutdown_pipe):
        while(True):
            idx, func, args = communication_pipe.recv()
            if(shutdown_pipe.poll()):
                break
            # Perform Functionality
            result  = func(*args)
            communication_pipe.send([idx, result])
    
    def map(self, function ,inputs):
        pipe_inputs = []
        pipe_outputs  = []
        for idx,input in enumerate(inputs):
            pipe_inputs.append([idx,function,input])
        while(len(pipe_outputs)<len(inputs)):
            for idx in range(len(self.pipes)):
                if(len(pipe_inputs) > 0 and self.process_status[idx] == 1):
                    self.pipes[idx][0].send(pipe_inputs.pop())     
                    self.process_status[idx] = 0   
                if(self.pipes[idx][0].poll()):
                    pipe_outputs.append(self.pipes[idx][0].recv())     
                    self.process_status[idx] = 1
        pipe_outputs.sort(key=lambda x:x[0])
        final_output = [item[1] for item in pipe_outputs]  
        return final_output 
    
    def shutdown(self):
        self.shutdown_pipe.send(True)
        for pipe in self.pipes:
            parent_connection = pipe[0]
            parent_connection.send((False,False,False))
        for process in self.processes:
            process.join()
            
if __name__ == "__main__":
    pool = FastPool(16)
    pool.shutdown()