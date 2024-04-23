import sys, random, enum, ast, time, threading, os, math, contextlib
from rpy2 import robjects
    
def R_to_Py_plot_priority(people, smoke, location, image_name):
    r_script = (f'''
                data <- read_excel("data/moral_sensitivity_survey_data.xlsx")
                data$situation <- as.factor(data$situation)
                data$location <- as.factor(data$location)
                data$smoke <- as.factor(data$smoke)
                data_subset <- subset(data, data$situation=="3"|data$situation=="6")
                data_subset <- data_subset[data_subset$smoke != "pushing out",]
                data_subset$people <- as.numeric(data_subset$people)
                fit <- lm(sensitivity ~ people + smoke + location, data = data_subset[-c(293, 205, 199, 162, 144, 122, 94, 76, 74, 18),])
                pred_data <- subset(data_subset[-c(293, 205, 199, 162, 144, 122, 94, 76, 74, 18),], select = c("people", "smoke", "location", "sensitivity"))
                pred_data$smoke <- factor(pred_data$smoke, levels = c("fast", "normal", "slow"))
                explainer <- shapr(pred_data, fit)
                p <- mean(pred_data$sensitivity)
                new_data <- data.frame(people = c({people}),
                                    smoke = c("{smoke}"),
                                    location = c("{location}"))
                new_data$smoke <- factor(new_data$smoke, levels = c("fast", "normal", "slow"))
                new_data$location <- factor(new_data$location, levels = c("known", "unknown"))
                #new_pred <- predict(fit, new_data)
                explanation_cat <- shapr::explain(new_data, approach = "ctree", explainer = explainer, prediction_zero = p)
                pl <- plot(explanation_cat, digits = 1, plot_phi0 = TRUE) 
                data_plot <- pl[["data"]]
                data_plot$phi <- round(data_plot$phi, 1)
                new_pred <- round(sum(data_plot$phi), 1)
                min <- 'min.'
                loc <- NA
                if ("{location}" == 'known') {{
                    loc <- 'found'
                }}
                if ("{location}" == 'unknown') {{
                    loc <- '?'
                }}
                labels <- c(none = "<br> baseline <br> moral <br> sensitivity", 
                smoke = paste("<img src='/home/ruben/xai4mhc/Icons/smoke_speed_black.png' width='53' /><br>\n", new_data$smoke), 
                location = paste("<img src='/home/ruben/xai4mhc/Icons/location_fire_black.png' width='35' /><br>\n", loc), 
                people = paste("<img src='/home/ruben/xai4mhc/Icons/victims.png' width='19' /><br>\n", new_data$people))
                idx <- 1
                original_rank <- data_plot$rank[idx]
                data_plot$rank[data_plot$rank < original_rank] <- data_plot$rank[data_plot$rank < original_rank] + 1
                data_plot$rank[idx] <- 1
                order_indices <- order(data_plot$rank)
                data_plot <- data_plot[order_indices, ]
                data_plot$variable <- factor(data_plot$variable, levels = unique(data_plot$variable))
                pl <- ggplot(data_plot, aes(x = variable, y = phi, fill = ifelse(data_plot$variable == "none", "#3E6F9F", ifelse(data_plot$phi >= 0, "#dc4c5d", "#117733")))) +
                geom_bar(stat = "identity") +
                geom_text(aes(label = ifelse(data_plot$variable=="none", phi, ifelse(data_plot$phi>=0, paste("+", phi, sep=""), paste("-", abs(phi)))),
                                y = ifelse(phi >= 0, phi + 0.1, phi - 0.1)), family = "sans-serif", color = "black", size = 4, 
                                vjust = ifelse(data_plot$phi >= 0, 0.4, 0.6)) +
                scale_x_discrete(name = NULL, labels = labels) +
                theme(axis.text.x = ggtext::element_markdown()) + # Removed color and size attributes here
                theme(text = element_text(size = 5, family = "sans-serif"), # Changed size to 12, which is roughly 1rem
                        plot.title = element_text(hjust = 0.5, size = 12, color = "black", face = "bold", margin = margin(b = 5)),
                        plot.caption = element_text(size = 12, margin = margin(t = 25), color = "black"),
                        panel.background = element_blank(),
                        axis.text = element_text(size = 12, color = "black"),
                        axis.text.y = element_text(color = "black", margin = margin(t = 5)),
                        axis.line = element_line(color = "black"),
                        axis.title = element_text(size = 12),
                        axis.title.y = element_text(color = "black", margin = margin(r = 10), hjust = 0.5),
                        axis.title.x = element_text(color = "black", margin = margin(t = 5), hjust = 0.5),
                        panel.grid.major = element_line(color = "#DAE1E7"),
                        panel.grid.major.x = element_blank()) +
                theme(legend.background = element_rect(fill = "white", color = "white"),
                        legend.key = element_rect(fill = "white", color = "white"),
                        legend.text = element_text(size = 12),
                        legend.position = "none",
                        legend.title = element_text(size = 12, face = "plain")) +
                labs(y = "Contribution to predicted sensitivity", fill = "") + 
                scale_fill_manual(values = c("#3E6F9F" = "#3E6F9F", "#117733" = "#117733", "#dc4c5d" = "#dc4c5d"), 
                                    labels = c("#3E6F9F" = "Baseline", "#117733" = "Increase", "#dc4c5d" = "Decrease")) + 
                geom_hline(yintercept = 0, color = "black") + 
                theme(axis.text = element_text(color = "black"),
                        axis.ticks = element_line(color = "black"))
                dpi_web <- 300
                width_pixels <- 1500
                height_pixels <- 1500
                width_inches_web <- width_pixels / dpi_web
                height_inches_web <- height_pixels / dpi_web
                ggsave(filename="{image_name}", plot=pl, width=width_inches_web, height=height_inches_web, dpi=dpi_web)
                ''')
    with open(os.devnull, 'w') as nullfile:
        with contextlib.redirect_stdout(nullfile), contextlib.redirect_stderr(nullfile):
            robjects.r(r_script)
    sensitivity = robjects.r['new_pred'][0]
    return sensitivity

def R_to_Py_plot_tactic(people, location, resistance, image_name):
    r_script = (f'''
                data <- read_excel("data/moral_sensitivity_survey_data.xlsx")
                data$situation <- as.factor(data$situation)
                data$location <- as.factor(data$location)
                data_subset <- subset(data, data$situation=="5"|data$situation=="7")
                data_subset$people[data_subset$people == "0"] <- "none"
                data_subset$people[data_subset$people == "1"] <- "one"
                data_subset$people[data_subset$people == "10" |data_subset$people == "11" |data_subset$people == "2" |data_subset$people == "3" |data_subset$people == "4" |data_subset$people == "5"] <- "multiple"
                data_subset <- data_subset[data_subset$people != "clear",]
                data_subset$people <- factor(data_subset$people, levels = c("none","unclear","one","multiple"))
                fit <- lm(sensitivity ~ people + resistance + location, data = data_subset[-c(266,244,186,178,126,111,97,44,19),])
                pred_data <- subset(data_subset[-c(266,244,186,178,126,111,97,44,19),], select = c("people", "resistance", "location", "sensitivity"))
                explainer <- shapr(pred_data, fit)
                p <- mean(pred_data$sensitivity)
                new_data <- data.frame(people = c("{people}"),
                                        resistance = c({resistance}),
                                        location = c("{location}"))
                new_data$people <- factor(new_data$people, levels = c("none", "unclear", "one", "multiple"))
                new_data$location <- factor(new_data$location, levels = c("known", "unknown"))
                #new_pred <- predict(fit, new_data)
                explanation_cat <- shapr::explain(new_data, approach = "ctree", explainer = explainer, prediction_zero = p)
                pl <- plot(explanation_cat, digits = 1, plot_phi0 = TRUE) 
                data_plot <- pl[["data"]]
                data_plot$phi <- round(data_plot$phi, 1)
                new_pred <- round(sum(data_plot$phi), 1)
                min <- 'min.'
                loc <- NA
                if ("{location}" == 'known') {{
                    loc <- 'found'
                }}
                if ("{location}" == 'unknown') {{
                    loc <- '?'
                }}
                labels <- c(none = "<br> baseline <br> moral <br> sensitivity", 
                resistance = paste("<img src='/home/ruben/xai4mhc/Icons/fire_resistance_black.png' width='38' /><br>\n", new_data$resistance, min), 
                location = paste("<img src='/home/ruben/xai4mhc/Icons/location_fire_black.png' width='35' /><br>\n", loc), 
                people = paste("<img src='/home/ruben/xai4mhc/Icons/victims.png' width='19' /><br>\n", new_data$people))
                idx <- 1
                original_rank <- data_plot$rank[idx]
                data_plot$rank[data_plot$rank < original_rank] <- data_plot$rank[data_plot$rank < original_rank] + 1
                data_plot$rank[idx] <- 1
                order_indices <- order(data_plot$rank)
                data_plot <- data_plot[order_indices, ]
                data_plot$variable <- factor(data_plot$variable, levels = unique(data_plot$variable))
                pl <- ggplot(data_plot, aes(x = variable, y = phi, fill = ifelse(data_plot$variable == "none", "#3E6F9F", ifelse(data_plot$phi >= 0, "#dc4c5d", "#117733")))) +
                geom_bar(stat = "identity") +
                geom_text(aes(label = ifelse(data_plot$variable=="none", phi, ifelse(data_plot$phi>=0, paste("+", phi, sep=""), paste("-", abs(phi)))),
                                y = ifelse(phi >= 0, phi + 0.1, phi - 0.1)), family = "sans-serif", color = "black", size = 4, 
                                vjust = ifelse(data_plot$phi >= 0, 0.4, 0.6)) +
                scale_x_discrete(name = NULL, labels = labels) +
                theme(axis.text.x = ggtext::element_markdown()) + # Removed color and size attributes here
                theme(text = element_text(size = 5, family = "sans-serif"), # Changed size to 12, which is roughly 1rem
                        plot.title = element_text(hjust = 0.5, size = 12, color = "black", face = "bold", margin = margin(b = 5)),
                        plot.caption = element_text(size = 12, margin = margin(t = 25), color = "black"),
                        panel.background = element_blank(),
                        axis.text = element_text(size = 12, color = "black"),
                        axis.text.y = element_text(color = "black", margin = margin(t = 5)),
                        axis.line = element_line(color = "black"),
                        axis.title = element_text(size = 12),
                        axis.title.y = element_text(color = "black", margin = margin(r = 10), hjust = 0.5),
                        axis.title.x = element_text(color = "black", margin = margin(t = 5), hjust = 0.5),
                        panel.grid.major = element_line(color = "#DAE1E7"),
                        panel.grid.major.x = element_blank()) +
                theme(legend.background = element_rect(fill = "white", color = "white"),
                        legend.key = element_rect(fill = "white", color = "white"),
                        legend.text = element_text(size = 12),
                        legend.position = "none",
                        legend.title = element_text(size = 12, face = "plain")) +
                labs(y = "Contribution to predicted sensitivity", fill = "") + 
                scale_fill_manual(values = c("#3E6F9F" = "#3E6F9F", "#117733" = "#117733", "#dc4c5d" = "#dc4c5d"), 
                                    labels = c("#3E6F9F" = "Baseline", "#117733" = "Increase", "#dc4c5d" = "Decrease")) + 
                geom_hline(yintercept = 0, color = "black") + 
                theme(axis.text = element_text(color = "black"),
                        axis.ticks = element_line(color = "black"))
                dpi_web <- 300
                width_pixels <- 1500
                height_pixels <- 1500
                width_inches_web <- width_pixels / dpi_web
                height_inches_web <- height_pixels / dpi_web
                ggsave(filename="{image_name}", plot=pl, width=width_inches_web, height=height_inches_web, dpi=dpi_web)
                ''')
    with open(os.devnull, 'w') as nullfile:
        with contextlib.redirect_stdout(nullfile), contextlib.redirect_stderr(nullfile):
            robjects.r(r_script)
    sensitivity = robjects.r['new_pred'][0]
    return sensitivity

    
def R_to_Py_plot_locate(people, resistance, temperature, image_name):
    r_script = (f'''
                data <- read_excel("data/moral_sensitivity_survey_data.xlsx")
                data$situation <- as.factor(data$situation)
                data$temperature <- as.factor(data$temperature)
                data_subset <- subset(data, data$situation=="2"|data$situation=="4")
                data_subset$people[data_subset$people == "0"] <- "none"
                data_subset$people[data_subset$people == "1"] <- "one"
                data_subset$people[data_subset$people == "10" |data_subset$people == "11" |data_subset$people == "2" |data_subset$people == "3" |data_subset$people == "4" |data_subset$people == "40" |data_subset$people == "5"] <- "multiple"
                data_subset <- data_subset[data_subset$people != "clear",]
                data_subset$people <- factor(data_subset$people, levels = c("none","unclear","one","multiple"))
                data_subset <- data_subset %>% drop_na(duration)
                fit <- lm(sensitivity ~ people + resistance + temperature, data = data_subset[-c(220, 195, 158, 126, 121, 76),])
                pred_data <- subset(data_subset[-c(220, 195, 158, 126, 121, 76),], select = c("people", "resistance", "temperature", "sensitivity"))
                explainer <- shapr(pred_data, fit)
                p <- mean(pred_data$sensitivity)
                new_data <- data.frame(resistance = c({resistance}),
                                        temperature = c("{temperature}"),
                                        people = c("{people}"))
                new_data$temperature <- factor(new_data$temperature, levels = c("close", "higher", "lower"))
                new_data$people <- factor(new_data$people, levels = c("none", "unclear", "one", "multiple"))
                #new_pred <- predict(fit, new_data)
                explanation_cat <- shapr::explain(new_data, approach = "ctree", explainer = explainer, prediction_zero = p)
                pl <- plot(explanation_cat, digits = 1, plot_phi0 = TRUE) 
                data_plot <- pl[["data"]]
                data_plot$phi <- round(data_plot$phi, 1)
                new_pred <- round(sum(data_plot$phi), 1)
                min <- 'min.'
                temp <- NA
                if ("{temperature}" == 'close') {{
                    temp <- '<≈ thresh.'
                }}
                if ("{temperature}" == 'lower') {{
                    temp <- '&lt; thresh.'
                }}
                if ("{temperature}" == 'higher') {{
                    temp <- '&gt; thresh.'
                }}
                labels <- c(none = "<br> baseline <br> moral <br> sensitivity", 
                resistance = paste("<img src='/home/ruben/xai4mhc/Icons/fire_resistance_black.png' width='38' /><br>\n", new_data$resistance, min), 
                temperature = paste("<img src='/home/ruben/xai4mhc/Icons/celsius_transparent.png' width='43' /><br>\n", temp), 
                people = paste("<img src='/home/ruben/xai4mhc/Icons/victims.png' width='19' /><br>\n", new_data$people))
                idx <- 1
                original_rank <- data_plot$rank[idx]
                data_plot$rank[data_plot$rank < original_rank] <- data_plot$rank[data_plot$rank < original_rank] + 1
                data_plot$rank[idx] <- 1
                order_indices <- order(data_plot$rank)
                data_plot <- data_plot[order_indices, ]
                data_plot$variable <- factor(data_plot$variable, levels = unique(data_plot$variable))
                pl <- ggplot(data_plot, aes(x = variable, y = phi, fill = ifelse(data_plot$variable == "none", "#3E6F9F", ifelse(data_plot$phi >= 0, "#dc4c5d", "#117733")))) +
                geom_bar(stat = "identity") +
                geom_text(aes(label = ifelse(data_plot$variable=="none", phi, ifelse(data_plot$phi>=0, paste("+", phi, sep=""), paste("-", abs(phi)))),
                                y = ifelse(phi >= 0, phi + 0.1, phi - 0.1)), family = "sans-serif", color = "black", size = 4, 
                                vjust = ifelse(data_plot$phi >= 0, 0.4, 0.6)) +
                scale_x_discrete(name = NULL, labels = labels) +
                theme(axis.text.x = ggtext::element_markdown()) + # Removed color and size attributes here
                theme(text = element_text(size = 5, family = "sans-serif"), # Changed size to 12, which is roughly 1rem
                        plot.title = element_text(hjust = 0.5, size = 12, color = "black", face = "bold", margin = margin(b = 5)),
                        plot.caption = element_text(size = 12, margin = margin(t = 25), color = "black"),
                        panel.background = element_blank(),
                        axis.text = element_text(size = 12, color = "black"),
                        axis.text.y = element_text(color = "black", margin = margin(t = 5)),
                        axis.line = element_line(color = "black"),
                        axis.title = element_text(size = 12),
                        axis.title.y = element_text(color = "black", margin = margin(r = 10), hjust = 0.5),
                        axis.title.x = element_text(color = "black", margin = margin(t = 5), hjust = 0.5),
                        panel.grid.major = element_line(color = "#DAE1E7"),
                        panel.grid.major.x = element_blank()) +
                theme(legend.background = element_rect(fill = "white", color = "white"),
                        legend.key = element_rect(fill = "white", color = "white"),
                        legend.text = element_text(size = 12),
                        legend.position = "none",
                        legend.title = element_text(size = 12, face = "plain")) +
                labs(y = "Contribution to predicted sensitivity", fill = "") + 
                scale_fill_manual(values = c("#3E6F9F" = "#3E6F9F", "#117733" = "#117733", "#dc4c5d" = "#dc4c5d"), 
                                    labels = c("#3E6F9F" = "Baseline", "#117733" = "Increase", "#dc4c5d" = "Decrease")) + 
                geom_hline(yintercept = 0, color = "black") + 
                theme(axis.text = element_text(color = "black"),
                        axis.ticks = element_line(color = "black"))
                dpi_web <- 300
                width_pixels <- 1500
                height_pixels <- 1500
                width_inches_web <- width_pixels / dpi_web
                height_inches_web <- height_pixels / dpi_web
                ggsave(filename="{image_name}", plot=pl, width=width_inches_web, height=height_inches_web, dpi=dpi_web)
                ''')
    with open(os.devnull, 'w') as nullfile:
        with contextlib.redirect_stdout(nullfile), contextlib.redirect_stderr(nullfile):
            robjects.r(r_script)
    sensitivity = robjects.r['new_pred'][0]
    return sensitivity

def R_to_Py_plot_rescue(resistance, temperature, distance, image_name):
    r_script = (f'''
                data <- read_excel("data/moral_sensitivity_survey_data.xlsx")
                data$situation <- as.factor(data$situation)
                data$temperature <- as.factor(data$temperature)
                data$distance <- as.factor(data$distance)
                data_subset <- subset(data, data$situation=="1"|data$situation=="8")
                data_subset$people <- as.numeric(data_subset$people)
                data_subset <- subset(data_subset, (!data_subset$temperature=="close"))
                data_subset <- data_subset %>% drop_na(distance)
                fit <- lm(sensitivity ~ resistance + temperature + distance, data = data_subset[-c(240, 237, 235, 202, 193, 121, 114, 108, 34, 28, 22),])
                pred_data <- subset(data_subset[-c(240, 237, 235, 202, 193, 121, 114, 108, 34, 28, 22),], select = c("resistance", "temperature", "distance", "sensitivity"))
                pred_data$temperature <- factor(pred_data$temperature, levels = c("higher", "lower"))
                explainer <- shapr(pred_data, fit)
                p <- mean(pred_data$sensitivity)
                new_data <- data.frame(resistance = c({resistance}),
                                        temperature = c("{temperature}"),
                                        distance = c("{distance}"))

                new_data$temperature <- factor(new_data$temperature, levels = c("higher", "lower"))
                new_data$distance <- factor(new_data$distance, levels = c("large", "small"))
                #new_pred <- predict(fit, new_data)
                explanation_cat <- shapr::explain(new_data, approach = "ctree", explainer = explainer, prediction_zero = p)
                pl <- plot(explanation_cat, digits = 1, plot_phi0 = TRUE) 
                levels(pl[["data"]]$sign) <- c("positive", "negative")
                data_plot <- pl[["data"]]
                data_plot$phi <- round(data_plot$phi, 1)
                new_pred <- round(sum(data_plot$phi), 1)
                min <- 'min.'
                temp <- NA
                if ("{temperature}" == 'close') {{
                    temp <- '<≈ thresh.'
                }}
                if ("{temperature}" == 'lower') {{
                    temp <- '&lt; thresh.'
                }}
                if ("{temperature}" == 'higher') {{
                    temp <- '&gt; thresh.'
                }}
                labels <- c(none = "<br> baseline <br> moral <br> sensitivity", 
                resistance = paste("<img src='/home/ruben/xai4mhc/Icons/fire_resistance_black.png' width='38' /><br>\n", new_data$resistance, min), 
                temperature = paste("<img src='/home/ruben/xai4mhc/Icons/celsius_transparent.png' width='43' /><br>\n", temp), 
                distance = paste("<img src='/home/ruben/xai4mhc/Icons/distance_fire_victim_black.png' width='54' /><br>\n", new_data$distance))
                idx <- 1
                original_rank <- data_plot$rank[idx]
                data_plot$rank[data_plot$rank < original_rank] <- data_plot$rank[data_plot$rank < original_rank] + 1
                data_plot$rank[idx] <- 1
                order_indices <- order(data_plot$rank)
                data_plot <- data_plot[order_indices, ]
                data_plot$variable <- factor(data_plot$variable, levels = unique(data_plot$variable))
                pl <- ggplot(data_plot, aes(x = variable, y = phi, fill = ifelse(data_plot$variable == "none", "#3E6F9F", ifelse(data_plot$phi >= 0, "#dc4c5d", "#117733")))) +
                geom_bar(stat = "identity") +
                geom_text(aes(label = ifelse(data_plot$variable=="none", phi, ifelse(data_plot$phi>=0, paste("+", phi, sep=""), paste("-", abs(phi)))),
                                y = ifelse(phi >= 0, phi + 0.1, phi - 0.1)), family = "sans-serif", color = "black", size = 4, 
                                vjust = ifelse(data_plot$phi >= 0, 0.4, 0.6)) +
                scale_x_discrete(name = NULL, labels = labels) +
                theme(axis.text.x = ggtext::element_markdown()) + # Removed color and size attributes here
                theme(text = element_text(size = 5, family = "sans-serif"), # Changed size to 12, which is roughly 1rem
                        plot.title = element_text(hjust = 0.5, size = 12, color = "black", face = "bold", margin = margin(b = 5)),
                        plot.caption = element_text(size = 12, margin = margin(t = 25), color = "black"),
                        panel.background = element_blank(),
                        axis.text = element_text(size = 12, color = "black"),
                        axis.text.y = element_text(color = "black", margin = margin(t = 5)),
                        axis.line = element_line(color = "black"),
                        axis.title = element_text(size = 12),
                        axis.title.y = element_text(color = "black", margin = margin(r = 10), hjust = 0.5),
                        axis.title.x = element_text(color = "black", margin = margin(t = 5), hjust = 0.5),
                        panel.grid.major = element_line(color = "#DAE1E7"),
                        panel.grid.major.x = element_blank()) +
                theme(legend.background = element_rect(fill = "white", color = "white"),
                        legend.key = element_rect(fill = "white", color = "white"),
                        legend.text = element_text(size = 12),
                        legend.position = "none",
                        legend.title = element_text(size = 12, face = "plain")) +
                labs(y = "Contribution to predicted sensitivity", fill = "") + 
                scale_fill_manual(values = c("#3E6F9F" = "#3E6F9F", "#117733" = "#117733", "#dc4c5d" = "#dc4c5d"), 
                                    labels = c("#3E6F9F" = "Baseline", "#117733" = "Increase", "#dc4c5d" = "Decrease")) + 
                geom_hline(yintercept = 0, color = "black") + 
                theme(axis.text = element_text(color = "black"),
                        axis.ticks = element_line(color = "black"))
                dpi_web <- 300
                width_pixels <- 1500
                height_pixels <- 1500
                width_inches_web <- width_pixels / dpi_web
                height_inches_web <- height_pixels / dpi_web
                ggsave(filename="{image_name}", plot=pl, width=width_inches_web, height=height_inches_web, dpi=dpi_web)
                ''')
    with open(os.devnull, 'w') as nullfile:
        with contextlib.redirect_stdout(nullfile), contextlib.redirect_stderr(nullfile):
            robjects.r(r_script)
    sensitivity = robjects.r['new_pred'][0]
    return sensitivity
    
# move to utils file and call once when running main.py
def load_R_to_Py():
    r_script = ('''
                # List of required packages
                required_packages <- c("readxl", "ggplot2", "dplyr", "rstatix", "ggpubr", "tidyverse", "gvlma", "lme4", "shapr", "ggtext", "ggdist")
                
                # Check if each package is installed, if not, install it
                for (package in required_packages) {
                    if (!(package %in% installed.packages())) {
                        install.packages(package, dependencies = TRUE)
                    }
                }
                # Load libraries
                library('readxl')
                library('ggplot2')
                library('dplyr')
                library('rstatix')
                library('ggpubr')
                library('tidyverse')
                library("gvlma")
                library('lme4')
                library('shapr')
                library('ggtext')
                library('ggdist')
                ''')
    robjects.r(r_script)
    
def add_object(locs, image, size, opacity, name):
    action_kwargs = {}
    add_objects = []
    for loc in locs:
        obj_kwargs = {}
        obj_kwargs['location'] = loc
        obj_kwargs['img_name'] = image
        obj_kwargs['visualize_size'] = size
        obj_kwargs['visualize_opacity'] = opacity
        obj_kwargs['name'] = name
        add_objects+=[obj_kwargs]
    action_kwargs['add_objects'] = add_objects
    return action_kwargs

def calculate_distances(p1, p2):
    # Unpack the coordinates
    x1, y1 = p1
    x2, y2 = p2
    
    # Euclidean distance
    euclidean_distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Manhattan distance
    manhattan_distance = abs(x2 - x1) + abs(y2 - y1)
    
    return euclidean_distance