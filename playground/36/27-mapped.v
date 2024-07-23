// Benchmark "adder" written by ABC on Thu Jul 18 06:38:30 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n342, new_n345, new_n347, new_n348,
    new_n350;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n16x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n09x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(new_n98), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aob012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nand02aa1d08x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor042aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n02x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  norp02aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand02aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n02x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nanp03aa1n02x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  aoi012aa1d18x5               g014(.a(new_n104), .b(new_n106), .c(new_n103), .o1(new_n110));
  xorc02aa1n02x5               g015(.a(\a[6] ), .b(\b[5] ), .out0(new_n111));
  xorc02aa1n02x5               g016(.a(\a[5] ), .b(\b[4] ), .out0(new_n112));
  nand42aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand42aa1n06x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norp02aa1n06x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nano23aa1n03x7               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  nand03aa1n02x5               g022(.a(new_n117), .b(new_n111), .c(new_n112), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  nor042aa1n03x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  aob012aa1n03x5               g025(.a(new_n120), .b(\b[5] ), .c(\a[6] ), .out0(new_n121));
  oaib12aa1n02x5               g026(.a(new_n121), .b(\b[5] ), .c(new_n119), .out0(new_n122));
  oai012aa1n02x5               g027(.a(new_n115), .b(new_n116), .c(new_n114), .o1(new_n123));
  aobi12aa1n09x5               g028(.a(new_n123), .b(new_n117), .c(new_n122), .out0(new_n124));
  aoai13aa1n09x5               g029(.a(new_n124), .b(new_n118), .c(new_n109), .d(new_n110), .o1(new_n125));
  nand42aa1n16x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n16x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand02aa1n03x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  nor042aa1n04x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand42aa1n08x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nano23aa1d18x5               g038(.a(new_n97), .b(new_n132), .c(new_n133), .d(new_n126), .out0(new_n134));
  tech160nm_fiao0012aa1n02p5x5 g039(.a(new_n132), .b(new_n97), .c(new_n133), .o(new_n135));
  aoai13aa1n02x5               g040(.a(new_n131), .b(new_n135), .c(new_n125), .d(new_n134), .o1(new_n136));
  aoi112aa1n02x5               g041(.a(new_n131), .b(new_n135), .c(new_n125), .d(new_n134), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g043(.a(new_n129), .o1(new_n139));
  nor002aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n136), .c(new_n139), .out0(\s[12] ));
  aoi012aa1n12x5               g048(.a(new_n98), .b(new_n100), .c(new_n101), .o1(new_n144));
  nanb02aa1n09x5               g049(.a(new_n104), .b(new_n103), .out0(new_n145));
  inv040aa1d32x5               g050(.a(\a[3] ), .o1(new_n146));
  inv000aa1d42x5               g051(.a(\b[2] ), .o1(new_n147));
  nand02aa1n02x5               g052(.a(new_n147), .b(new_n146), .o1(new_n148));
  nand02aa1n03x5               g053(.a(new_n148), .b(new_n107), .o1(new_n149));
  nor043aa1n03x5               g054(.a(new_n144), .b(new_n145), .c(new_n149), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n110), .o1(new_n151));
  xnrc02aa1n02x5               g056(.a(\b[5] ), .b(\a[6] ), .out0(new_n152));
  xnrc02aa1n02x5               g057(.a(\b[4] ), .b(\a[5] ), .out0(new_n153));
  nona23aa1n09x5               g058(.a(new_n115), .b(new_n113), .c(new_n116), .d(new_n114), .out0(new_n154));
  nor043aa1n03x5               g059(.a(new_n154), .b(new_n153), .c(new_n152), .o1(new_n155));
  oai012aa1n06x5               g060(.a(new_n155), .b(new_n150), .c(new_n151), .o1(new_n156));
  nano23aa1n06x5               g061(.a(new_n129), .b(new_n140), .c(new_n141), .d(new_n130), .out0(new_n157));
  nand22aa1n09x5               g062(.a(new_n157), .b(new_n134), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n130), .b(new_n132), .c(new_n97), .d(new_n133), .o1(new_n159));
  nor042aa1n02x5               g064(.a(new_n140), .b(new_n129), .o1(new_n160));
  aob012aa1n02x5               g065(.a(new_n141), .b(new_n159), .c(new_n160), .out0(new_n161));
  aoai13aa1n03x5               g066(.a(new_n161), .b(new_n158), .c(new_n156), .d(new_n124), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nanp02aa1n04x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  aoi012aa1n02x5               g070(.a(new_n164), .b(new_n162), .c(new_n165), .o1(new_n166));
  xnrb03aa1n02x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1d12x5               g072(.a(new_n110), .b(new_n144), .c(new_n145), .d(new_n149), .o1(new_n168));
  oaib12aa1n03x5               g073(.a(new_n123), .b(new_n154), .c(new_n122), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n158), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n169), .c(new_n168), .d(new_n155), .o1(new_n171));
  nor002aa1d32x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand02aa1d28x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nona23aa1n09x5               g078(.a(new_n173), .b(new_n165), .c(new_n164), .d(new_n172), .out0(new_n174));
  aoi012aa1d24x5               g079(.a(new_n172), .b(new_n164), .c(new_n173), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n174), .c(new_n171), .d(new_n161), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g082(.a(\a[15] ), .o1(new_n178));
  nanb02aa1d36x5               g083(.a(\b[14] ), .b(new_n178), .out0(new_n179));
  nano23aa1n06x5               g084(.a(new_n164), .b(new_n172), .c(new_n173), .d(new_n165), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n175), .o1(new_n181));
  nand02aa1d08x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nanp02aa1n24x5               g087(.a(new_n179), .b(new_n182), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n181), .c(new_n162), .d(new_n180), .o1(new_n185));
  xnrc02aa1n12x5               g090(.a(\b[15] ), .b(\a[16] ), .out0(new_n186));
  tech160nm_fiaoi012aa1n02p5x5 g091(.a(new_n186), .b(new_n185), .c(new_n179), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n179), .o1(new_n188));
  xorc02aa1n02x5               g093(.a(\a[16] ), .b(\b[15] ), .out0(new_n189));
  aoi112aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n176), .d(new_n182), .o1(new_n190));
  norp02aa1n02x5               g095(.a(new_n187), .b(new_n190), .o1(\s[16] ));
  nand03aa1n02x5               g096(.a(new_n180), .b(new_n184), .c(new_n189), .o1(new_n192));
  nor042aa1n06x5               g097(.a(new_n192), .b(new_n158), .o1(new_n193));
  aoai13aa1n06x5               g098(.a(new_n193), .b(new_n169), .c(new_n168), .d(new_n155), .o1(new_n194));
  aoi022aa1n06x5               g099(.a(new_n159), .b(new_n160), .c(\b[11] ), .d(\a[12] ), .o1(new_n195));
  nor003aa1n06x5               g100(.a(new_n174), .b(new_n183), .c(new_n186), .o1(new_n196));
  inv000aa1d42x5               g101(.a(\a[16] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(\b[15] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(new_n198), .b(new_n197), .o1(new_n199));
  and002aa1n02x5               g104(.a(\b[15] ), .b(\a[16] ), .o(new_n200));
  aoai13aa1n12x5               g105(.a(new_n182), .b(new_n172), .c(new_n164), .d(new_n173), .o1(new_n201));
  aoai13aa1n09x5               g106(.a(new_n199), .b(new_n200), .c(new_n201), .d(new_n179), .o1(new_n202));
  aoi012aa1n12x5               g107(.a(new_n202), .b(new_n196), .c(new_n195), .o1(new_n203));
  nor042aa1d18x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  nand42aa1d28x5               g109(.a(\b[16] ), .b(\a[17] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n194), .c(new_n203), .out0(\s[17] ));
  nand03aa1n02x5               g112(.a(new_n196), .b(new_n134), .c(new_n157), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n203), .b(new_n208), .c(new_n156), .d(new_n124), .o1(new_n209));
  tech160nm_fiaoi012aa1n05x5   g114(.a(new_n204), .b(new_n209), .c(new_n206), .o1(new_n210));
  xnrb03aa1n03x5               g115(.a(new_n210), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n09x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nand42aa1d28x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nano23aa1d15x5               g118(.a(new_n204), .b(new_n212), .c(new_n213), .d(new_n205), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoi012aa1n06x5               g120(.a(new_n212), .b(new_n204), .c(new_n213), .o1(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n215), .c(new_n194), .d(new_n203), .o1(new_n217));
  xorb03aa1n03x5               g122(.a(new_n217), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nand02aa1n08x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  xnrc02aa1n12x5               g126(.a(\b[19] ), .b(\a[20] ), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoi112aa1n03x4               g128(.a(new_n220), .b(new_n223), .c(new_n217), .d(new_n221), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n220), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n216), .o1(new_n226));
  nanb02aa1n18x5               g131(.a(new_n220), .b(new_n221), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n226), .c(new_n209), .d(new_n214), .o1(new_n229));
  aoi012aa1n03x5               g134(.a(new_n222), .b(new_n229), .c(new_n225), .o1(new_n230));
  norp02aa1n03x5               g135(.a(new_n230), .b(new_n224), .o1(\s[20] ));
  nona22aa1d18x5               g136(.a(new_n214), .b(new_n222), .c(new_n227), .out0(new_n232));
  aoai13aa1n06x5               g137(.a(new_n221), .b(new_n212), .c(new_n204), .d(new_n213), .o1(new_n233));
  oab012aa1n04x5               g138(.a(new_n220), .b(\a[20] ), .c(\b[19] ), .out0(new_n234));
  aoi022aa1n12x5               g139(.a(new_n233), .b(new_n234), .c(\b[19] ), .d(\a[20] ), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoai13aa1n04x5               g141(.a(new_n236), .b(new_n232), .c(new_n194), .d(new_n203), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  nand42aa1n06x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  norb02aa1n02x5               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  nor002aa1d32x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nand42aa1n06x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nanb02aa1n12x5               g148(.a(new_n242), .b(new_n243), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoi112aa1n03x4               g150(.a(new_n239), .b(new_n245), .c(new_n237), .d(new_n241), .o1(new_n246));
  inv000aa1n02x5               g151(.a(new_n239), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n232), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n241), .b(new_n235), .c(new_n209), .d(new_n248), .o1(new_n249));
  aoi012aa1n03x5               g154(.a(new_n244), .b(new_n249), .c(new_n247), .o1(new_n250));
  norp02aa1n03x5               g155(.a(new_n250), .b(new_n246), .o1(\s[22] ));
  nano23aa1n06x5               g156(.a(new_n239), .b(new_n242), .c(new_n243), .d(new_n240), .out0(new_n252));
  nona23aa1d16x5               g157(.a(new_n214), .b(new_n252), .c(new_n222), .d(new_n227), .out0(new_n253));
  nand22aa1n04x5               g158(.a(new_n233), .b(new_n234), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(\b[19] ), .b(\a[20] ), .o1(new_n255));
  nano32aa1n03x7               g160(.a(new_n244), .b(new_n247), .c(new_n240), .d(new_n255), .out0(new_n256));
  oaoi03aa1n02x5               g161(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .o1(new_n257));
  aoi012aa1n02x5               g162(.a(new_n257), .b(new_n254), .c(new_n256), .o1(new_n258));
  aoai13aa1n04x5               g163(.a(new_n258), .b(new_n253), .c(new_n194), .d(new_n203), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  tech160nm_fixorc02aa1n04x5   g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  xorc02aa1n12x5               g167(.a(\a[24] ), .b(\b[23] ), .out0(new_n263));
  aoi112aa1n03x4               g168(.a(new_n261), .b(new_n263), .c(new_n259), .d(new_n262), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n261), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n253), .o1(new_n266));
  inv000aa1n02x5               g171(.a(new_n258), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n262), .b(new_n267), .c(new_n209), .d(new_n266), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n263), .o1(new_n269));
  tech160nm_fiaoi012aa1n02p5x5 g174(.a(new_n269), .b(new_n268), .c(new_n265), .o1(new_n270));
  nor002aa1n02x5               g175(.a(new_n270), .b(new_n264), .o1(\s[24] ));
  nano32aa1n03x7               g176(.a(new_n232), .b(new_n263), .c(new_n252), .d(new_n262), .out0(new_n272));
  inv000aa1n02x5               g177(.a(new_n272), .o1(new_n273));
  and002aa1n06x5               g178(.a(new_n263), .b(new_n262), .o(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n257), .c(new_n254), .d(new_n256), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .carry(new_n276));
  nand02aa1d10x5               g181(.a(new_n275), .b(new_n276), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n273), .c(new_n194), .d(new_n203), .o1(new_n279));
  xorb03aa1n02x5               g184(.a(new_n279), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[25] ), .b(\b[24] ), .out0(new_n282));
  xorc02aa1n12x5               g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  aoi112aa1n03x4               g188(.a(new_n281), .b(new_n283), .c(new_n279), .d(new_n282), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n281), .o1(new_n285));
  aoai13aa1n03x5               g190(.a(new_n282), .b(new_n277), .c(new_n209), .d(new_n272), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n283), .o1(new_n287));
  tech160nm_fiaoi012aa1n05x5   g192(.a(new_n287), .b(new_n286), .c(new_n285), .o1(new_n288));
  nor042aa1n03x5               g193(.a(new_n288), .b(new_n284), .o1(\s[26] ));
  nanp02aa1n02x5               g194(.a(new_n201), .b(new_n179), .o1(new_n290));
  oaoi03aa1n02x5               g195(.a(new_n197), .b(new_n198), .c(new_n290), .o1(new_n291));
  oaih12aa1n02x5               g196(.a(new_n291), .b(new_n161), .c(new_n192), .o1(new_n292));
  and002aa1n02x5               g197(.a(new_n283), .b(new_n282), .o(new_n293));
  nano22aa1d15x5               g198(.a(new_n253), .b(new_n293), .c(new_n274), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n292), .c(new_n125), .d(new_n193), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(\b[25] ), .b(\a[26] ), .o1(new_n296));
  oai022aa1n02x5               g201(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n297));
  aoi022aa1n12x5               g202(.a(new_n277), .b(new_n293), .c(new_n296), .d(new_n297), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  xnbna2aa1n03x5               g204(.a(new_n299), .b(new_n298), .c(new_n295), .out0(\s[27] ));
  norp02aa1n02x5               g205(.a(\b[26] ), .b(\a[27] ), .o1(new_n301));
  inv040aa1n03x5               g206(.a(new_n301), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n299), .o1(new_n303));
  aoi012aa1n03x5               g208(.a(new_n303), .b(new_n298), .c(new_n295), .o1(new_n304));
  xnrc02aa1n12x5               g209(.a(\b[27] ), .b(\a[28] ), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n304), .b(new_n302), .c(new_n305), .out0(new_n306));
  inv000aa1n02x5               g211(.a(new_n293), .o1(new_n307));
  nanp02aa1n02x5               g212(.a(new_n297), .b(new_n296), .o1(new_n308));
  aoai13aa1n04x5               g213(.a(new_n308), .b(new_n307), .c(new_n275), .d(new_n276), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n299), .b(new_n309), .c(new_n209), .d(new_n294), .o1(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n305), .b(new_n310), .c(new_n302), .o1(new_n311));
  norp02aa1n03x5               g216(.a(new_n311), .b(new_n306), .o1(\s[28] ));
  xnrc02aa1n02x5               g217(.a(\b[28] ), .b(\a[29] ), .out0(new_n313));
  norb02aa1n06x5               g218(.a(new_n299), .b(new_n305), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n309), .c(new_n209), .d(new_n294), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n316));
  aoi012aa1n02x7               g221(.a(new_n313), .b(new_n315), .c(new_n316), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n314), .o1(new_n318));
  aoi012aa1n03x5               g223(.a(new_n318), .b(new_n298), .c(new_n295), .o1(new_n319));
  nano22aa1n03x5               g224(.a(new_n319), .b(new_n313), .c(new_n316), .out0(new_n320));
  norp02aa1n03x5               g225(.a(new_n317), .b(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n06x5               g227(.a(new_n299), .b(new_n313), .c(new_n305), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n309), .c(new_n209), .d(new_n294), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n316), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .out0(new_n326));
  tech160nm_fiaoi012aa1n02p5x5 g231(.a(new_n326), .b(new_n324), .c(new_n325), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n323), .o1(new_n328));
  aoi012aa1n03x5               g233(.a(new_n328), .b(new_n298), .c(new_n295), .o1(new_n329));
  nano22aa1n03x5               g234(.a(new_n329), .b(new_n325), .c(new_n326), .out0(new_n330));
  norp02aa1n03x5               g235(.a(new_n327), .b(new_n330), .o1(\s[30] ));
  nona32aa1n02x4               g236(.a(new_n299), .b(new_n326), .c(new_n313), .d(new_n305), .out0(new_n332));
  aoi012aa1n02x7               g237(.a(new_n332), .b(new_n298), .c(new_n295), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n334));
  xnrc02aa1n02x5               g239(.a(\b[30] ), .b(\a[31] ), .out0(new_n335));
  nano22aa1n03x5               g240(.a(new_n333), .b(new_n334), .c(new_n335), .out0(new_n336));
  inv000aa1n02x5               g241(.a(new_n332), .o1(new_n337));
  aoai13aa1n02x5               g242(.a(new_n337), .b(new_n309), .c(new_n209), .d(new_n294), .o1(new_n338));
  aoi012aa1n02x5               g243(.a(new_n335), .b(new_n338), .c(new_n334), .o1(new_n339));
  norp02aa1n03x5               g244(.a(new_n339), .b(new_n336), .o1(\s[31] ));
  xnbna2aa1n03x5               g245(.a(new_n144), .b(new_n107), .c(new_n148), .out0(\s[3] ));
  oaoi03aa1n02x5               g246(.a(\a[3] ), .b(\b[2] ), .c(new_n144), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g248(.a(new_n168), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g249(.a(new_n120), .b(new_n168), .c(new_n112), .o1(new_n345));
  xorb03aa1n02x5               g250(.a(new_n345), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  norp02aa1n02x5               g251(.a(new_n153), .b(new_n152), .o1(new_n347));
  tech160nm_fiao0012aa1n02p5x5 g252(.a(new_n122), .b(new_n168), .c(new_n347), .o(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g254(.a(new_n116), .b(new_n348), .c(new_n113), .o1(new_n350));
  xnrb03aa1n02x5               g255(.a(new_n350), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g256(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


