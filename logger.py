import time

def log(message):
    current_time = time.time()
    local_time = time.localtime(current_time)
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S', local_time)
    log_message = "{}: {}".format(timestamp, message)  # Format the log message

    with open('log.txt', 'a') as log_file:
        log_file.write(log_message + '\n')
        print(log_message)  # Print the log message to console

# Example usage:
if __name__ == "__main__":
    log("This is a log message.")