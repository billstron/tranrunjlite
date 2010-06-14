/*
 * Copyright (c) 2009, Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *  * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *  * Neither the name of the University of California, Berkeley
 * nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package TranRunJLite;

import java.util.GregorianCalendar;

/**A time class for TranRunJLite that operates in real time.  This also
 * functions as the protptype for other timer objects for TrjLite.
 *
 * @author WJBurke
 */
public interface TrjTime {

    /** Get the Gregorian Calendar for the current time.
     *
     * @return -- Current date/time
     */
    public GregorianCalendar getCalendar();

    /** Get the Gregorian Calendar associated with a specified runtime.
     *
     * @param t -- runtime of desired calendar
     * @return date/time of this time
     */
    public GregorianCalendar getCalendar(double t);

    /** Get the current time in runtime
     *
     * @return time (s) of the runtime
     */
    public double getRunningTime();

    /** Step the time forward by the value specified
     *
     * @param dt amount to increment the time by (s)
     */
    public void incrementRunningTime(double dt);

    /** Reset the start times to the current time.  
     */
    public void resetRunningTime();
}
