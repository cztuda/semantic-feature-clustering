#pragma once
#pragma once

#include <lwpr.h>
#include <lwpr_aux.h>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <iostream>

namespace boost{
    namespace serialization {
        
        namespace helper {
            template<class Archive, typename T>
            void matrix_rw(Archive& ar, int M, int Ms, int N, T data){
                for (int n=0; n<N; ++n) {
                    for(int i=0; i<M; ++i) {
                        ar & data[n*Ms + i];
                    }
                }
            }
            
            template<class Archive, typename T>
            void vector_rw(Archive& ar, int N, T data) {
                for(int i=0; i<N; ++i) {
                    ar & data[i];
                }
            }
        }
        
        /**
         * SERIALIZE LWPR_RECEPTIVEFIELD
         */
        template<class Archive, typename T>
        void rw_RF(Archive& ar, T rf, int nIn, int nInS, int nReg){ 
            helper::matrix_rw(ar, nIn, nInS, nIn, rf->D);
            helper::matrix_rw(ar, nIn, nInS, nIn, rf->M);
            helper::matrix_rw(ar, nIn, nInS, nIn, rf->alpha);
            ar & rf->beta0;
            helper::vector_rw(ar, nReg, rf->beta);
            helper::vector_rw(ar, nIn, rf->c);
            helper::matrix_rw(ar, nIn, nInS, nReg, rf->SXresYres);
            helper::vector_rw(ar, nReg, rf->SSs2);
            helper::vector_rw(ar, nReg, rf->SSYres);
            helper::matrix_rw(ar, nIn, nInS, nReg, rf->SSXres);
            helper::matrix_rw(ar, nIn, nInS, nReg, rf->U);
            helper::matrix_rw(ar, nIn, nInS, nReg, rf->P);
            helper::vector_rw(ar, nReg, rf->H);
            helper::vector_rw(ar, nReg, rf->r);
            helper::matrix_rw(ar, nIn, nInS, nIn, rf->h);
            helper::matrix_rw(ar, nIn, nInS, nIn, rf->b);
            helper::vector_rw(ar, nReg, rf->sum_w);
            helper::vector_rw(ar, nReg, rf->sum_e_cv2);
            ar & rf->sum_e2;
            ar & rf->SSp;
            helper::vector_rw(ar, nReg, rf->n_data);
            ar & rf->trustworthy;
            helper::vector_rw(ar, nReg, rf->lambda);
            helper::vector_rw(ar, nIn, rf->mean_x);
            helper::vector_rw(ar, nIn, rf->var_x);
            ar & rf->w;
            helper::vector_rw(ar, nReg, rf->s);
        }
        
        
        /**
         * SERIALIZE LWPR_MODEL
         */
        
        template<class Archive>
        void save(Archive& ar, const LWPR_Model& model, unsigned int){ // is it correct that const is missing here?
            int nIn = model.nIn;
            int nOut = model.nOut;
            int nInS = model.nInStore;
            std::string name;
            ar & model.nIn;
            ar & model.nOut;
            ar & static_cast<int>(model.kernel);
            if(model.name == nullptr){
                ar & std::string("");
            }
            else {
                ar & std::string(model.name);
            }
            ar & model.n_data;
            helper::vector_rw(ar, nIn, model.mean_x);
            helper::vector_rw(ar, nIn, model.var_x);
            ar & model.diag_only;
            ar & model.update_D;
            ar & model.meta;
            ar & model.meta_rate;
            ar & model.penalty;
            helper::matrix_rw(ar, nIn, nInS, nIn, model.init_alpha);
            helper::vector_rw(ar, nIn, model.norm_in);
            helper::vector_rw(ar, nOut, model.norm_out);
            helper::matrix_rw(ar, nIn, nInS, nIn, model.init_D);
            helper::matrix_rw(ar, nIn, nInS, nIn, model.init_M);
            ar & model.w_gen;
            ar & model.w_prune;
            ar & model.init_lambda;
            ar & model.final_lambda;
            ar & model.tau_lambda;
            ar & model.init_S2;
            ar & model.add_threshold;
            
            for(int dim = 0; dim < model.nOut; dim++){
                LWPR_SubModel* sub = &model.sub[dim];
                ar & model.sub->numRFS;
                ar & model.sub->n_pruned;
                for(int i = 0; i < sub->numRFS; ++i){
                    ar & sub->rf[i]->nReg;
                    rw_RF<Archive, const LWPR_ReceptiveField*>(ar, sub->rf[i], model.nIn, model.nInStore, sub->rf[i]->nReg);
                }
            }
        }
        template<class Archive>
        void load(Archive& ar, LWPR_Model& model, unsigned int){
            int nIn, nOut, kernel;
            std::string name;
            ar & nIn;
            ar & nOut;            
            lwpr_init_model(&model, nIn, nOut, NULL);
            int nInS = model.nInStore;
            
            ar & kernel;
            model.kernel = static_cast<LWPR_Kernel>(kernel);
            ar & name;
            size_t len = name.size();
            model.name = new char[len+1];
            if(model.name != NULL){
                strncpy(model.name, name.c_str(), len);
                model.name[len] = '\0';
            }
            ar & model.n_data;
            helper::vector_rw(ar, nIn, model.mean_x);
            helper::vector_rw(ar, nIn, model.var_x);
            ar & model.diag_only;
            ar & model.update_D;
            ar & model.meta;
            ar & model.meta_rate;
            ar & model.penalty;
            helper::matrix_rw(ar, nIn, nInS, nIn, model.init_alpha);
            helper::vector_rw(ar, nIn, model.norm_in);
            helper::vector_rw(ar, nOut, model.norm_out);
            helper::matrix_rw(ar, nIn, nInS, nIn, model.init_D);
            helper::matrix_rw(ar, nIn, nInS, nIn, model.init_M);
            ar & model.w_gen;
            ar & model.w_prune;
            ar & model.init_lambda;
            ar & model.final_lambda;
            ar & model.tau_lambda;
            ar & model.init_S2;
            ar & model.add_threshold;
            
            int nReg, numRFS;
            LWPR_ReceptiveField* RF;
            for(int dim = 0; dim < model.nOut; dim++){
                LWPR_SubModel* sub = &model.sub[dim];
                ar & numRFS;
                ar & model.sub->n_pruned;
                for(int i = 0; i < numRFS; ++i){
                    ar & nReg;
                    RF = lwpr_aux_add_rf(sub, nReg);
                    rw_RF<Archive, LWPR_ReceptiveField*>(ar, RF, model.nIn, model.nInStore, nReg);
                    sub->rf[i] = RF;
                }
            }
        }
        
    } // namespace serialization
} // namespace boost


// BOOST_SERIALIZATION_SPLIT_FREE(LWPR_ReceptiveField)
BOOST_SERIALIZATION_SPLIT_FREE(LWPR_Model)
