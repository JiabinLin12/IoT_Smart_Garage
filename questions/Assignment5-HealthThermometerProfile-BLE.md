Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.*

1. Provide screen shot verifying the Advertising period matches the values required for the assignment.
   <br>Screenshot:  
   ![advertising_period](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/Period.png)  

2. What is the average current between advertisements?
   Answer:2.02uA
   <br>Screenshot:  
   ![avg_current_between_advertisements](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/avg_bt_avert.png)  

3. What is the peak current of an advertisement? 
   Answer:27.07mA
   <br>Screenshot:  
   ![peak_current_of_advertisement](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/avert_peak_current.png)  

4. Provide screen shot showing the connection interval settings. Do they match the values you set in your slave(server) code or the master's(client) values?.
   <br>Screenshot: 
   ![connection_interval](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/param.png)  
   
   They do not match my setting in the slave code. However,it matches the connection interval period set on the master value
   <br>Screenshot: 
   ![connection_interval_profile](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/cont_intv.png) 

5. What is the average current between connection intervals?
   Answer:1.68uA
   <br>Screenshot:  
   ![avg_current_between_connection_intervals](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/avg_bt_cont_intv.png)  

6. If possible, provide screen shot verifying the slave latency matches what was reported when you logged the values from event = gecko_evt_le_connection_parameters_id. 
   
   My client set the slave latency to 0, which means it does not skip any of the connection interval. In the picture, we can see that the indication transmission power is a lot bigger(24mA) than the regular connection interval(18mA), this is a good indication that the bluetooth transmitted at this connection interval. This connection event happens right after the indication is queued, so we verify that BT does not miss any connection interval.
   <br>Screenshot:  
   ![slave_latency](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/slave_latency.png)  

7. What is the peak current of a data transmission when the phone is connected and placed next to the Blue Gecko? 
   Answer:20.48mA
   <br>Screenshot:  
   ![peak_current_phone_next_to](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/pwr_0ft.png)  
   
8. What is the peak current of a data transmission when the phone is connected and placed approximately 20 feet away from the Blue Gecko? 
   Answer:24.17mA
   <br>Screenshot:  
   ![peak_current_phone_20ft_away](https://github.com/CU-ECEN-5823/ecen5823-assignment5-JiabinLin12/blob/master/img/A5/pwr_20ft.png)  
   
