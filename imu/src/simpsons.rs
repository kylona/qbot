use core::f32;


pub struct SimpsonsIntegral {
    partial_sum : f32,
    pub result : f32,
    delta_t : f32,
    buffer : [f32; 2],
}
impl SimpsonsIntegral {
    pub fn default() -> Self {
        Self::new(1.0/500.0, 0.0)
    }
    pub fn new(delta_t : f32, init_sum : f32) -> Self {
        let buffer = [init_sum/delta_t; 2];
        Self {
            partial_sum: 0.0,
            result: init_sum,
            delta_t: delta_t,
            buffer: buffer,
        }
    }
    pub fn update(&mut self, value : f32) -> f32 {
        let update_sum = self.delta_t/3.0 * (self.buffer[0] + 4.0*self.buffer[1] + value);
        self.result = self.partial_sum + update_sum;
        self.partial_sum += self.delta_t * (self.buffer[0] + self.buffer[1])/2.0;
        self.buffer[0] = self.buffer[1];
        self.buffer[1] = value;
        self.result
    }
    pub fn get_result(&self) -> f32 {
        self.result
    }
}


    #[test]
    fn test_simpsons_constant() {
        let mut simpsons = SimpsonsIntegral::new(1.0/500.0, 1.0);
        assert_eq!(simpsons.get_result(), 1.0, "Simpsons Init value incorrect");
        assert_eq!(simpsons.update(500.0), 2.0, "Update 1 incorrect");
        simpsons.update(500.0);
        assert_eq!(simpsons.update(500.0), 4.0, "Update 3 incorrect");
    }

    #[test]
    fn test_sin() {
        let mut simpsons = SimpsonsIntegral::new(1.0/500.0, 0.0);
        for i in 0..=100 {
            let theta = (i as f32)/100.0 * (2.0*f32::consts::PI);
            println!("S: {}", simpsons.update(500.0*f32::sin(theta)));
        }
    }

    #[test]
    fn test_const_then_zero() {
        let mut simpsons = SimpsonsIntegral::new(1.0/500.0, 0.0);
        for _ in 0..10 {
            println!("C: {}", simpsons.update(500.0));
        }
        for _ in 0..100 {
            println!("Z: {}", simpsons.update(0.0));
        }
    }