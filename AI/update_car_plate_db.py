import csv
import pymysql
from collections import defaultdict
from datetime import datetime, timedelta
from DB_SET import DB_SET

def process_csv(file_path):
    license_plates = defaultdict(list)
    with open(file_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            license_plate = row['License Plate Text']
            timestamp = datetime.strptime(row['Capture Time'], '%Y-%m-%d %H:%M:%S')
            license_plates[license_plate].append(timestamp)
    return license_plates

def update_database(data):
    connection = pymysql.connect(
        host=DB_SET['HOST'],
        user=DB_SET['USER'],
        password=DB_SET['PASSWORD'],
        database=DB_SET['DATABASE']
    )

    try:
        with connection.cursor() as cursor:
            for plate, timestamps in data.items():
                for timestamp in timestamps:
                    cursor.execute("SELECT IN_TIME, OUT_TIME FROM CAR_INFO WHERE CAR_NUM = %s", (plate,))
                    result = cursor.fetchone()
                    
                    if result:
                        in_time, out_time = result
                        if in_time and not out_time:
                            # Check if the new OUT_TIME is at least 2 minutes after IN_TIME
                            if timestamp - in_time >= timedelta(minutes=2):
                                cursor.execute("UPDATE CAR_INFO SET OUT_TIME = %s AND PK_NUM = %s WHERE CAR_NUM = %s", (timestamp, "O1", plate))
                                
                        else:
                            # Insert new record only if the time difference is at least 2 minutes
                            if in_time and timestamp - in_time >= timedelta(minutes=2):
                                cursor.execute("INSERT INTO CAR_INFO (CAR_NUM, IN_TIME, PK_NUM) VALUES (%s, %s, %s)", (plate, timestamp, "I1"))
                            elif not in_time:
                                cursor.execute("INSERT INTO CAR_INFO (CAR_NUM, IN_TIME, PK_NUM) VALUES (%s, %s, %s)", (plate, timestamp, "I1"))
                    else:
                        # No existing record, just insert the new record
                        cursor.execute("INSERT INTO CAR_INFO (CAR_NUM, IN_TIME, PK_NUM) VALUES (%s, %s, %s)", (plate, timestamp, "I1"))
                        
        
        connection.commit()
        print("Data has been updated successfully.")
    
    except Exception as e:
        print(f"Update error: {e}")
    
    finally:
        connection.close()

def remove_redundant():
    connection = pymysql.connect(
        host=DB_SET['HOST'],
        user=DB_SET['USER'],
        password=DB_SET['PASSWORD'],
        database=DB_SET['DATABASE']
    )
    
    try:
        with connection.cursor() as cursor:
            create_temp_table_query = """
            CREATE TEMPORARY TABLE temp_table AS
            SELECT CAR_NUM, MIN(IN_TIME) AS min_in_time
            FROM CAR_INFO
            GROUP BY CAR_NUM;
            """
            
            delete_query = """
            DELETE FROM CAR_INFO
            WHERE (CAR_NUM, IN_TIME) NOT IN (
                SELECT CAR_NUM, min_in_time
                FROM temp_table
            );
            """
            
            cursor.execute(create_temp_table_query)
            cursor.execute(delete_query)
            connection.commit()
            print("Redundant data has been successfully deleted.")
    
    except Exception as e:
        print(f"Deletion error: {e}")
    
    finally:
        connection.close()

if __name__ == "__main__":
    file_path = "AI/car_plate_detection/data/detected_license_plates.csv"  # Saved car_plate_info CSV file path
    processed_data = process_csv(file_path)
    update_database(processed_data)
    remove_redundant()
