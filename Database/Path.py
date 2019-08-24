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

        # ******************************************************************************************************************************
        # for tracking test
        logging_file = list()

        f = open("C:\\Users\\HEVEN\\Documents\\GitHub\\HEVEN-AutonomousCar-2019\\DB-Team\\Real-World\\Database\\gps_park_test.txt", 'r')
        while True:
            line = f.readline()
            if not line: break
            temp = line.replace('[', '').replace(']', '').split(',')

            temp[1] = float(temp[1])
            temp[3] = float(temp[3])
            m_lad = temp[1] % 100
            d_lad = (temp[1] - m_lad)/100
            temp[1] = (d_lad + m_lad/60)*110000

            m_lon = temp[3] % 100
            d_lon = (temp[3] - m_lon)/100
            temp[3] = (d_lon + m_lon/60)*88800

            logging_file.append([float(temp[1]), float(temp[3])])
            
        f.close()

        self.gps_path_test_for_Control = logging_file

        logging_file = list()

        f = open("C:/Users/HEVEN/Desktop/K-City 예선전 경로 파일(LLH).txt", 'r')
        while True:
            line = f.readline()
            if not line: break
            temp = line.split('\t')
            logging_file.append([float(temp[0])*110000, float(temp[1])*88800])
        f.close()

        self.gps_1_track = logging_file

        logging_file = list()

        f = open("C:/Users/HEVEN/Desktop/K-City 본선전 경로 파일(LLH).txt", 'r')
        while True:
            line = f.readline()
            if not line: break
            temp = line.split('\t')
            logging_file.append([float(temp[0])*110000, float(temp[1])*88800])
        f.close()

        self.gps_2_track = logging_file