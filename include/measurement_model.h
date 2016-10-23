#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

namespace str
{
	class MeasurementModel;
}

namespace str
{
	class MeasurementModel
	{
		public:
			//tuning parameters for sensor model(TBD)
			struct tuningParameters
			{
				double w_hit = 1.0;
				double w_short = 1.0;
				double w_max = 1.0;
				double w_rand = 1.0;
				double sigma_hit = 1.0;
				double lambda_short = 1.0;
			};

			MeasurementModel();
			virtual ~MeasurementModel();
			double getProbability();
			void setParameters(tuningParameters& input_param);
			
		private:
			double getProbFromBeamModel(const double& measurement, const double& predict_measurement);
			double prob_hit(const double& measurement, const double& predict_measurement);
			double prob_short(const double& measurement, const double& predict_measurement);
			double prob_max(const double& measurement);
			double prob_rand(const double& measurement);

			tuningParameters params_;
			double z_max_;//need to discuss how to get this one.

		
		
	};

}


#endif //MEASUREMENT_MODEL_H
