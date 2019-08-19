class Path:
    def __init__(self):
        logging_file = list()

        f = open("C:\\Users\\HEVEN\\Documents\\GitHub\\HEVEN-AutonomousCar-2019\\DB-Team\\Real-World\\Database\\gps.txt", 'r')
        while True:
            line = f.readline()
            if not line: break
            temp = line.replace('[', '').replace(']', '').split(',')
            logging_file.append([3000 * float(temp[1]), 3000 * float(temp[3])])
        f.close()

        self.gps_pre_path = logging_file

        logging_file = list()

        f = open("C:\\Users\\HEVEN\\Documents\\GitHub\\HEVEN-AutonomousCar-2019\\DB-Team\\Real-World\\Database\\gps2.txt", 'r')
        while True:
            line = f.readline()
            if not line: break
            temp = line.replace('[', '').replace(']', '').split(',')
            logging_file.append([3000 * float(temp[1]), 3000 * float(temp[3])])
        f.close()

        self.gps_main_path = logging_file
        self.generate_path = None

        logging_file = list()

        f = open("C:\\Users\\HEVEN\\Documents\\GitHub\\HEVEN-AutonomousCar-2019\\DB-Team\\Real-World\\Database\\gps_track_test.txt", 'r')
        while True:
            line = f.readline()
            if not line: break
            temp = line.replace('[', '').replace(']', '').split(',')
            logging_file.append([float(temp[1]), float(temp[3])])
        f.close()

        self.gps_path_test_for_Control = logging_file