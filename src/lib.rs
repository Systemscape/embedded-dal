#![no_std]

pub mod backends;
pub mod drivers;
pub mod test_images;

pub mod config {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum Orientation {
        Landscape,
        Portrait,
    }

    pub struct Dimensions {
        x: u16,
        y: u16,
    }

    impl Dimensions {
        pub const fn from(x: u16, y: u16) -> Self {
            Self { x, y }
        }
        pub const fn get_width(&self, orientation: Orientation) -> u16 {
            match orientation {
                Orientation::Landscape => self.x,
                Orientation::Portrait => self.y,
            }
        }

        pub const fn get_height(&self, orientation: Orientation) -> u16 {
            match orientation {
                Orientation::Landscape => self.y,
                Orientation::Portrait => self.x,
            }
        }
    }
}

pub mod pixels {
    #[repr(C)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, Default)]
    pub struct ARGB8888 {
        pub b: u8,
        pub g: u8,
        pub r: u8,
        pub a: u8,
    }

    impl ARGB8888 {
        pub const fn new(a: u8, r: u8, g: u8, b: u8) -> Self {
            Self { b, g, r, a }
        }

        pub const fn new_rgb(r: u8, g: u8, b: u8) -> Self {
            Self::new(0xFF, r, g, b)
        }
    }

    //#[repr(packed(1))]
    #[repr(align(1))]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Clone, Copy, Debug, Default)]
    pub struct RGB888 {
        pub b: u8,
        pub g: u8,
        pub r: u8,
    }

    impl RGB888 {
        pub const fn new(r: u8, g: u8, b: u8) -> Self {
            Self { b, g, r }
        }
    }
}

pub mod slint_compat {
    impl slint::platform::software_renderer::TargetPixel for crate::pixels::RGB888 {
        fn blend(&mut self, color: PremultipliedRgbaColor) {
            let a = (u8::MAX - color.alpha) as u16;
            self.r = (self.r as u16 * a / 255) as u8 + color.red;
            self.g = (self.g as u16 * a / 255) as u8 + color.green;
            self.b = (self.b as u16 * a / 255) as u8 + color.blue;
        }

        fn from_rgb(r: u8, g: u8, b: u8) -> Self {
            Self::new(r, g, b)
        }
    }

    use slint::platform::software_renderer::PremultipliedRgbaColor;

    impl slint::platform::software_renderer::TargetPixel for crate::pixels::ARGB8888 {
        fn blend(&mut self, color: PremultipliedRgbaColor) {
            let a = (u8::MAX - color.alpha) as u16;
            self.r = (self.r as u16 * a / 255) as u8 + color.red;
            self.g = (self.g as u16 * a / 255) as u8 + color.green;
            self.b = (self.b as u16 * a / 255) as u8 + color.blue;
            self.a = 0xFF;
        }

        fn from_rgb(r: u8, g: u8, b: u8) -> Self {
            Self::new(0xFF, r, g, b)
        }
    }
}
