import matplotlib.pyplot as plt
import gmplot
import numpy as np
import rosbag


#log_filename = 'g11/g11_city_data.bag'
#log_filename = 'ack_only_filt.bag'
log_filename = 'ack_full_speed_partial.bag'

data_dict = {'timestamp': [],'latitude': [],'longitude': [],'altitude': [],'easting': [],'northing': [],'zone': []}

bag = rosbag.Bag(log_filename)
#for topic, msg, t in bag.read_messages(topics=['/vehicle/gps/fix']):
#	data_dict['latitude'].append(msg.latitude)
#	data_dict['longitude'].append(msg.longitude)
for topic, msg, t in bag.read_messages(topics=['/eece5554/odom_filtered_gps']):
#for topic, msg, t in bag.read_messages(topics=['/vehicle/gps/fix']):
	data_dict['latitude'].append(msg.latitude)
	data_dict['longitude'].append(msg.longitude)

bag.close()

gmap = gmplot.GoogleMapPlotter(np.mean(data_dict['latitude']), np.mean(data_dict['longitude']), 18)
gmap.apikey = 'AIzaSyB04sktSVt62tEOcrFFvfQhKf53rRmoeEc'

gmap.scatter(data_dict['latitude'], data_dict['longitude'], '#0493FF', size = 1, marker = False)
gmap.plot(data_dict['latitude'], data_dict['longitude'], 'cornflowerblue', edge_width = 1)

gmap.draw("map-ack_full_speed_partial.html")


# plt.plot(data_dict['easting'], data_dict['northing'], 'o')
# plt.show()
