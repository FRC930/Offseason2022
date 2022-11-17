package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;

//  private class interface
    //  -- structure to hold data for each target
    public class TargetInfo {
        //-----------------------------------------------------
        //  Public objects to use where needed
        //  Create Static Enum to use for target referencing
        public static enum Targets {
            RedSide("RedPlayerStation"),
            BlueSide("BluePlayerStation"),
            TopField("TopField"),
            BottomField("BottomField"),
            RedTarget("RedTarget"),
            BlueTarget("BlueTarget"),
            TopTarget("TopTarget"),
            BottomTarget("BottomTarget");

            public final String m_name;

            Targets(String name) {
                m_name = name;
            }
        }

        //  attributes
        private String m_Type ;
        private Pose2d m_Position ;

        //  constructor
        public TargetInfo (String vType, Pose2d vFieldPos) {
            m_Type = vType ;
            m_Position = vFieldPos ;
        }

        //
        //  getters
        public String getTargetType() {
            return m_Type ;
        }

        public Pose2d getTargetPos() {
            return m_Position ;
        }
    }
