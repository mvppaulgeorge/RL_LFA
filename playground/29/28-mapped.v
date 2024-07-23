// Benchmark "adder" written by ABC on Thu Jul 18 03:03:58 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n343, new_n346, new_n348,
    new_n350, new_n352;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  and002aa1n06x5               g002(.a(\b[0] ), .b(\a[1] ), .o(new_n98));
  oaoi03aa1n12x5               g003(.a(\a[2] ), .b(\b[1] ), .c(new_n98), .o1(new_n99));
  nand22aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  norb02aa1n03x5               g006(.a(new_n100), .b(new_n101), .out0(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n03x4               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nanp03aa1n06x5               g010(.a(new_n99), .b(new_n102), .c(new_n105), .o1(new_n106));
  aoi012aa1n02x5               g011(.a(new_n101), .b(new_n103), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nanp02aa1n06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor042aa1n03x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nano23aa1n02x4               g016(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n112));
  nor042aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  and002aa1n12x5               g018(.a(\b[7] ), .b(\a[8] ), .o(new_n114));
  nand22aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nor002aa1n03x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanb02aa1n03x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nona32aa1n02x4               g022(.a(new_n112), .b(new_n117), .c(new_n114), .d(new_n113), .out0(new_n118));
  norp03aa1n03x5               g023(.a(new_n114), .b(new_n116), .c(new_n113), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[6] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n110), .b(new_n109), .o1(new_n121));
  oaib12aa1n06x5               g026(.a(new_n121), .b(\b[5] ), .c(new_n120), .out0(new_n122));
  oab012aa1n02x4               g027(.a(new_n114), .b(new_n113), .c(new_n116), .out0(new_n123));
  aoi013aa1n03x5               g028(.a(new_n123), .b(new_n119), .c(new_n122), .d(new_n115), .o1(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n118), .c(new_n106), .d(new_n107), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanb02aa1n02x5               g031(.a(new_n97), .b(new_n126), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  aoi012aa1n02x5               g033(.a(new_n97), .b(new_n125), .c(new_n128), .o1(new_n129));
  xnrb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand22aa1n03x5               g035(.a(new_n106), .b(new_n107), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n109), .b(new_n108), .out0(new_n132));
  norb02aa1n02x5               g037(.a(new_n111), .b(new_n110), .out0(new_n133));
  norp02aa1n02x5               g038(.a(new_n114), .b(new_n113), .o1(new_n134));
  nano32aa1n03x7               g039(.a(new_n117), .b(new_n134), .c(new_n133), .d(new_n132), .out0(new_n135));
  nanp03aa1n02x5               g040(.a(new_n119), .b(new_n122), .c(new_n115), .o1(new_n136));
  oai012aa1n03x5               g041(.a(new_n136), .b(new_n119), .c(new_n114), .o1(new_n137));
  nor042aa1n04x5               g042(.a(\b[9] ), .b(\a[10] ), .o1(new_n138));
  nand02aa1d06x5               g043(.a(\b[9] ), .b(\a[10] ), .o1(new_n139));
  nano23aa1n02x4               g044(.a(new_n138), .b(new_n97), .c(new_n126), .d(new_n139), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n137), .c(new_n131), .d(new_n135), .o1(new_n141));
  aoi012aa1n02x5               g046(.a(new_n138), .b(new_n97), .c(new_n139), .o1(new_n142));
  norp02aa1n04x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nanp02aa1n04x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n141), .c(new_n142), .out0(\s[11] ));
  inv000aa1n02x5               g051(.a(new_n143), .o1(new_n147));
  aob012aa1n02x5               g052(.a(new_n145), .b(new_n141), .c(new_n142), .out0(new_n148));
  nor042aa1n04x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand02aa1n06x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n148), .c(new_n147), .out0(\s[12] ));
  nano23aa1n02x5               g057(.a(new_n143), .b(new_n149), .c(new_n150), .d(new_n144), .out0(new_n153));
  and002aa1n02x5               g058(.a(new_n153), .b(new_n140), .o(new_n154));
  aoai13aa1n03x5               g059(.a(new_n154), .b(new_n137), .c(new_n131), .d(new_n135), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n149), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n150), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n144), .b(new_n138), .c(new_n97), .d(new_n139), .o1(new_n158));
  aoai13aa1n12x5               g063(.a(new_n156), .b(new_n157), .c(new_n158), .d(new_n147), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  norp02aa1n04x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n03x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n155), .c(new_n160), .out0(\s[13] ));
  nanp02aa1n02x5               g069(.a(new_n155), .b(new_n160), .o1(new_n165));
  aoi012aa1n02x5               g070(.a(new_n161), .b(new_n165), .c(new_n162), .o1(new_n166));
  xnrb03aa1n02x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n02x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand42aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nano23aa1n02x4               g074(.a(new_n161), .b(new_n168), .c(new_n169), .d(new_n162), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n159), .c(new_n125), .d(new_n154), .o1(new_n171));
  tech160nm_fiaoi012aa1n05x5   g076(.a(new_n168), .b(new_n161), .c(new_n169), .o1(new_n172));
  nor022aa1n04x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1n03x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanb02aa1n12x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n171), .c(new_n172), .out0(\s[15] ));
  nona23aa1n03x5               g082(.a(new_n169), .b(new_n162), .c(new_n161), .d(new_n168), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n172), .b(new_n178), .c(new_n155), .d(new_n160), .o1(new_n179));
  xnrc02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n173), .c(new_n179), .d(new_n174), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n179), .b(new_n176), .o1(new_n182));
  nona22aa1n02x4               g087(.a(new_n182), .b(new_n180), .c(new_n173), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n181), .o1(\s[16] ));
  nona22aa1n03x5               g089(.a(new_n170), .b(new_n180), .c(new_n175), .out0(new_n185));
  nano22aa1n03x7               g090(.a(new_n185), .b(new_n140), .c(new_n153), .out0(new_n186));
  aoai13aa1n09x5               g091(.a(new_n186), .b(new_n137), .c(new_n131), .d(new_n135), .o1(new_n187));
  nor043aa1n03x5               g092(.a(new_n178), .b(new_n175), .c(new_n180), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\a[16] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[15] ), .o1(new_n190));
  tech160nm_fioaoi03aa1n03p5x5 g095(.a(\a[15] ), .b(\b[14] ), .c(new_n172), .o1(new_n191));
  oaib12aa1n03x5               g096(.a(new_n191), .b(new_n190), .c(\a[16] ), .out0(new_n192));
  oaib12aa1n02x5               g097(.a(new_n192), .b(\b[15] ), .c(new_n189), .out0(new_n193));
  aoi012aa1n12x5               g098(.a(new_n193), .b(new_n159), .c(new_n188), .o1(new_n194));
  nanp02aa1n12x5               g099(.a(new_n187), .b(new_n194), .o1(new_n195));
  nor002aa1n06x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  nand42aa1d28x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aoi112aa1n02x5               g103(.a(new_n193), .b(new_n198), .c(new_n159), .d(new_n188), .o1(new_n199));
  aoi022aa1n02x5               g104(.a(new_n195), .b(new_n198), .c(new_n199), .d(new_n187), .o1(\s[17] ));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n196), .b(new_n195), .c(new_n198), .o1(new_n201));
  xnrb03aa1n03x5               g106(.a(new_n201), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n04x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nand42aa1n20x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nano23aa1d15x5               g109(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  oa0012aa1n02x5               g111(.a(new_n204), .b(new_n203), .c(new_n196), .o(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n04x5               g113(.a(new_n208), .b(new_n206), .c(new_n187), .d(new_n194), .o1(new_n209));
  nor002aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand42aa1n06x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norb02aa1n03x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  aoi112aa1n02x5               g117(.a(new_n212), .b(new_n207), .c(new_n195), .d(new_n205), .o1(new_n213));
  aoi012aa1n02x5               g118(.a(new_n213), .b(new_n209), .c(new_n212), .o1(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n02x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand02aa1n03x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanb02aa1n02x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n210), .c(new_n209), .d(new_n211), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(new_n209), .b(new_n212), .o1(new_n220));
  nona22aa1n02x4               g125(.a(new_n220), .b(new_n218), .c(new_n210), .out0(new_n221));
  nanp02aa1n03x5               g126(.a(new_n221), .b(new_n219), .o1(\s[20] ));
  nanb03aa1n03x5               g127(.a(new_n216), .b(new_n217), .c(new_n211), .out0(new_n223));
  orn002aa1n02x5               g128(.a(\a[19] ), .b(\b[18] ), .o(new_n224));
  oai112aa1n03x5               g129(.a(new_n224), .b(new_n204), .c(new_n203), .d(new_n196), .o1(new_n225));
  inv000aa1n02x5               g130(.a(new_n216), .o1(new_n226));
  aob012aa1n02x5               g131(.a(new_n226), .b(new_n210), .c(new_n217), .out0(new_n227));
  oabi12aa1n12x5               g132(.a(new_n227), .b(new_n225), .c(new_n223), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  nanb03aa1d24x5               g134(.a(new_n218), .b(new_n205), .c(new_n212), .out0(new_n230));
  aoai13aa1n04x5               g135(.a(new_n229), .b(new_n230), .c(new_n187), .d(new_n194), .o1(new_n231));
  nor022aa1n04x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n230), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n234), .b(new_n195), .c(new_n235), .o1(new_n236));
  aoi022aa1n02x5               g141(.a(new_n236), .b(new_n229), .c(new_n231), .d(new_n234), .o1(\s[21] ));
  nor042aa1n02x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n232), .c(new_n231), .d(new_n234), .o1(new_n242));
  nand22aa1n03x5               g147(.a(new_n231), .b(new_n234), .o1(new_n243));
  nona22aa1n02x4               g148(.a(new_n243), .b(new_n241), .c(new_n232), .out0(new_n244));
  nanp02aa1n02x5               g149(.a(new_n244), .b(new_n242), .o1(\s[22] ));
  tech160nm_finand02aa1n03p5x5 g150(.a(new_n159), .b(new_n188), .o1(new_n246));
  oaoi03aa1n02x5               g151(.a(new_n189), .b(new_n190), .c(new_n191), .o1(new_n247));
  nand02aa1d04x5               g152(.a(new_n246), .b(new_n247), .o1(new_n248));
  nano23aa1n06x5               g153(.a(new_n232), .b(new_n238), .c(new_n239), .d(new_n233), .out0(new_n249));
  nano32aa1n02x4               g154(.a(new_n218), .b(new_n249), .c(new_n205), .d(new_n212), .out0(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n248), .c(new_n125), .d(new_n186), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n250), .o1(new_n252));
  nano22aa1n02x4               g157(.a(new_n216), .b(new_n211), .c(new_n217), .out0(new_n253));
  oai012aa1n02x5               g158(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .o1(new_n254));
  oab012aa1n03x5               g159(.a(new_n254), .b(new_n196), .c(new_n203), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n249), .b(new_n227), .c(new_n255), .d(new_n253), .o1(new_n256));
  oa0012aa1n02x5               g161(.a(new_n239), .b(new_n238), .c(new_n232), .o(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  nanp02aa1n04x5               g163(.a(new_n256), .b(new_n258), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n04x5               g165(.a(new_n260), .b(new_n252), .c(new_n187), .d(new_n194), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n262), .b(new_n257), .c(new_n228), .d(new_n249), .o1(new_n263));
  aoi022aa1n02x5               g168(.a(new_n261), .b(new_n262), .c(new_n251), .d(new_n263), .o1(\s[23] ));
  norp02aa1n02x5               g169(.a(\b[22] ), .b(\a[23] ), .o1(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .out0(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n261), .d(new_n262), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(new_n261), .b(new_n262), .o1(new_n268));
  nona22aa1n02x4               g173(.a(new_n268), .b(new_n266), .c(new_n265), .out0(new_n269));
  nanp02aa1n02x5               g174(.a(new_n269), .b(new_n267), .o1(\s[24] ));
  norb02aa1n03x5               g175(.a(new_n262), .b(new_n266), .out0(new_n271));
  inv040aa1n02x5               g176(.a(new_n271), .o1(new_n272));
  nona22aa1n02x4               g177(.a(new_n195), .b(new_n252), .c(new_n272), .out0(new_n273));
  nanp02aa1n02x5               g178(.a(new_n250), .b(new_n271), .o1(new_n274));
  orn002aa1n02x5               g179(.a(\a[23] ), .b(\b[22] ), .o(new_n275));
  oao003aa1n02x5               g180(.a(\a[24] ), .b(\b[23] ), .c(new_n275), .carry(new_n276));
  aoai13aa1n12x5               g181(.a(new_n276), .b(new_n272), .c(new_n256), .d(new_n258), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  aoai13aa1n04x5               g183(.a(new_n278), .b(new_n274), .c(new_n187), .d(new_n194), .o1(new_n279));
  tech160nm_fixorc02aa1n03p5x5 g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n271), .b(new_n257), .c(new_n228), .d(new_n249), .o1(new_n281));
  nano22aa1n02x4               g186(.a(new_n280), .b(new_n281), .c(new_n276), .out0(new_n282));
  aoi022aa1n02x5               g187(.a(new_n282), .b(new_n273), .c(new_n279), .d(new_n280), .o1(\s[25] ));
  norp02aa1n02x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n284), .c(new_n279), .d(new_n280), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(new_n279), .b(new_n280), .o1(new_n288));
  nona22aa1n02x4               g193(.a(new_n288), .b(new_n286), .c(new_n284), .out0(new_n289));
  nanp02aa1n02x5               g194(.a(new_n289), .b(new_n287), .o1(\s[26] ));
  and002aa1n24x5               g195(.a(new_n285), .b(new_n280), .o(new_n291));
  nano32aa1d15x5               g196(.a(new_n230), .b(new_n291), .c(new_n249), .d(new_n271), .out0(new_n292));
  aoai13aa1n12x5               g197(.a(new_n292), .b(new_n248), .c(new_n125), .d(new_n186), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n291), .o1(new_n295));
  inv000aa1d42x5               g200(.a(\a[26] ), .o1(new_n296));
  inv000aa1d42x5               g201(.a(\b[25] ), .o1(new_n297));
  oao003aa1n02x5               g202(.a(new_n296), .b(new_n297), .c(new_n284), .carry(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n295), .c(new_n281), .d(new_n276), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n195), .d(new_n292), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n301), .o1(new_n303));
  oai112aa1n02x5               g208(.a(new_n299), .b(new_n303), .c(new_n278), .d(new_n295), .o1(new_n304));
  oa0012aa1n03x5               g209(.a(new_n302), .b(new_n304), .c(new_n294), .o(\s[27] ));
  xnrc02aa1n02x5               g210(.a(\b[27] ), .b(\a[28] ), .out0(new_n306));
  aoi012aa1n12x5               g211(.a(new_n298), .b(new_n277), .c(new_n291), .o1(new_n307));
  nor042aa1n06x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  inv000aa1n03x5               g213(.a(new_n308), .o1(new_n309));
  aoai13aa1n04x5               g214(.a(new_n309), .b(new_n303), .c(new_n307), .d(new_n293), .o1(new_n310));
  nand02aa1n02x5               g215(.a(new_n310), .b(new_n306), .o1(new_n311));
  nona22aa1n02x5               g216(.a(new_n302), .b(new_n306), .c(new_n308), .out0(new_n312));
  nanp02aa1n03x5               g217(.a(new_n311), .b(new_n312), .o1(\s[28] ));
  norb02aa1n03x5               g218(.a(new_n301), .b(new_n306), .out0(new_n314));
  aoai13aa1n02x5               g219(.a(new_n314), .b(new_n300), .c(new_n195), .d(new_n292), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  oao003aa1n03x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n309), .carry(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n314), .o1(new_n319));
  aoai13aa1n04x5               g224(.a(new_n317), .b(new_n319), .c(new_n307), .d(new_n293), .o1(new_n320));
  aoi022aa1n03x5               g225(.a(new_n320), .b(new_n316), .c(new_n315), .d(new_n318), .o1(\s[29] ));
  xnrb03aa1n02x5               g226(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g227(.a(new_n306), .b(new_n301), .c(new_n316), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n300), .c(new_n195), .d(new_n292), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n323), .o1(new_n325));
  tech160nm_fioaoi03aa1n02p5x5 g230(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .o1(new_n326));
  inv000aa1d42x5               g231(.a(new_n326), .o1(new_n327));
  aoai13aa1n02x7               g232(.a(new_n327), .b(new_n325), .c(new_n307), .d(new_n293), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .out0(new_n329));
  norp02aa1n02x5               g234(.a(new_n326), .b(new_n329), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n328), .b(new_n329), .c(new_n324), .d(new_n330), .o1(\s[30] ));
  nanp03aa1n02x5               g236(.a(new_n314), .b(new_n316), .c(new_n329), .o1(new_n332));
  oabi12aa1n03x5               g237(.a(new_n332), .b(new_n294), .c(new_n300), .out0(new_n333));
  xorc02aa1n02x5               g238(.a(\a[31] ), .b(\b[30] ), .out0(new_n334));
  inv000aa1d42x5               g239(.a(\a[30] ), .o1(new_n335));
  inv000aa1d42x5               g240(.a(\b[29] ), .o1(new_n336));
  oabi12aa1n02x5               g241(.a(new_n334), .b(\a[30] ), .c(\b[29] ), .out0(new_n337));
  oaoi13aa1n02x5               g242(.a(new_n337), .b(new_n326), .c(new_n335), .d(new_n336), .o1(new_n338));
  oaoi03aa1n02x5               g243(.a(new_n335), .b(new_n336), .c(new_n326), .o1(new_n339));
  aoai13aa1n04x5               g244(.a(new_n339), .b(new_n332), .c(new_n307), .d(new_n293), .o1(new_n340));
  aoi022aa1n03x5               g245(.a(new_n333), .b(new_n338), .c(new_n340), .d(new_n334), .o1(\s[31] ));
  xorb03aa1n02x5               g246(.a(new_n99), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi112aa1n02x5               g247(.a(new_n103), .b(new_n102), .c(new_n99), .d(new_n105), .o1(new_n343));
  oaoi13aa1n02x5               g248(.a(new_n343), .b(new_n131), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g249(.a(new_n133), .b(new_n106), .c(new_n107), .out0(\s[5] ));
  aoi012aa1n02x5               g250(.a(new_n110), .b(new_n131), .c(new_n111), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[5] ), .c(new_n120), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g252(.a(new_n122), .b(new_n131), .c(new_n112), .o(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g254(.a(new_n116), .b(new_n348), .c(new_n115), .o1(new_n350));
  xnrc02aa1n02x5               g255(.a(new_n350), .b(new_n134), .out0(\s[8] ));
  aoi112aa1n02x5               g256(.a(new_n128), .b(new_n137), .c(new_n131), .d(new_n135), .o1(new_n352));
  aoi012aa1n02x5               g257(.a(new_n352), .b(new_n125), .c(new_n128), .o1(\s[9] ));
endmodule


