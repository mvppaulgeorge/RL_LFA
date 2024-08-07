// Benchmark "adder" written by ABC on Wed Jul 17 15:04:03 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n334, new_n337, new_n338, new_n339, new_n341,
    new_n343;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d08x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n09x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  tech160nm_fixnrc02aa1n04x5   g005(.a(\b[3] ), .b(\a[4] ), .out0(new_n101));
  nor042aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n12x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  norp03aa1n06x5               g009(.a(new_n101), .b(new_n100), .c(new_n104), .o1(new_n105));
  inv000aa1n06x5               g010(.a(new_n102), .o1(new_n106));
  oaoi03aa1n12x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .o1(new_n107));
  nor022aa1n08x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand42aa1n04x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n12x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .out0(new_n113));
  norp02aa1n09x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand42aa1n16x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanb02aa1n02x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n112), .b(new_n113), .c(new_n116), .o1(new_n117));
  oai012aa1n06x5               g022(.a(new_n117), .b(new_n105), .c(new_n107), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[5] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[4] ), .o1(new_n120));
  inv040aa1d30x5               g025(.a(new_n115), .o1(new_n121));
  aoi112aa1n09x5               g026(.a(new_n121), .b(new_n114), .c(new_n119), .d(new_n120), .o1(new_n122));
  inv000aa1n02x5               g027(.a(new_n122), .o1(new_n123));
  nanb02aa1n02x5               g028(.a(new_n108), .b(new_n109), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n110), .o1(new_n125));
  nano32aa1n02x4               g030(.a(new_n124), .b(new_n115), .c(new_n125), .d(new_n111), .out0(new_n126));
  oai012aa1n02x5               g031(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n127));
  aobi12aa1n06x5               g032(.a(new_n127), .b(new_n126), .c(new_n123), .out0(new_n128));
  nanp02aa1n06x5               g033(.a(new_n118), .b(new_n128), .o1(new_n129));
  tech160nm_fixorc02aa1n03p5x5 g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  nor042aa1n04x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  nor042aa1d18x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanp02aa1n24x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n15x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n131), .c(new_n129), .d(new_n130), .o1(new_n136));
  nona22aa1n02x4               g041(.a(new_n133), .b(new_n132), .c(new_n131), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n137), .c(new_n130), .d(new_n129), .o1(\s[10] ));
  xnrc02aa1n02x5               g043(.a(\b[8] ), .b(\a[9] ), .out0(new_n139));
  norb02aa1n02x5               g044(.a(new_n134), .b(new_n139), .out0(new_n140));
  oai012aa1n06x5               g045(.a(new_n133), .b(new_n132), .c(new_n131), .o1(new_n141));
  aob012aa1n03x5               g046(.a(new_n141), .b(new_n129), .c(new_n140), .out0(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  nand42aa1n04x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  aoi012aa1n02x5               g050(.a(new_n144), .b(new_n142), .c(new_n145), .o1(new_n146));
  norp02aa1n12x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nanp02aa1n04x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  norb02aa1n02x5               g054(.a(new_n145), .b(new_n144), .out0(new_n150));
  nona22aa1n02x4               g055(.a(new_n148), .b(new_n147), .c(new_n144), .out0(new_n151));
  tech160nm_fiao0012aa1n03p5x5 g056(.a(new_n151), .b(new_n142), .c(new_n150), .o(new_n152));
  oaih12aa1n02x5               g057(.a(new_n152), .b(new_n146), .c(new_n149), .o1(\s[12] ));
  inv000aa1n03x5               g058(.a(new_n107), .o1(new_n154));
  oai013aa1n06x5               g059(.a(new_n154), .b(new_n101), .c(new_n100), .d(new_n104), .o1(new_n155));
  oai013aa1n09x5               g060(.a(new_n127), .b(new_n112), .c(new_n122), .d(new_n121), .o1(new_n156));
  nona23aa1n09x5               g061(.a(new_n148), .b(new_n145), .c(new_n144), .d(new_n147), .out0(new_n157));
  nor003aa1n03x5               g062(.a(new_n157), .b(new_n135), .c(new_n139), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n156), .c(new_n155), .d(new_n117), .o1(new_n159));
  tech160nm_fioai012aa1n03p5x5 g064(.a(new_n148), .b(new_n147), .c(new_n144), .o1(new_n160));
  oai012aa1d24x5               g065(.a(new_n160), .b(new_n157), .c(new_n141), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nor042aa1n06x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nand42aa1d28x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nanb02aa1n18x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n166), .b(new_n159), .c(new_n162), .out0(\s[13] ));
  orn002aa1n02x5               g072(.a(\a[13] ), .b(\b[12] ), .o(new_n168));
  aoai13aa1n03x5               g073(.a(new_n166), .b(new_n161), .c(new_n129), .d(new_n158), .o1(new_n169));
  nor042aa1n04x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand42aa1n10x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  xobna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n168), .out0(\s[14] ));
  tech160nm_fioai012aa1n03p5x5 g078(.a(new_n171), .b(new_n170), .c(new_n163), .o1(new_n174));
  nano23aa1d15x5               g079(.a(new_n163), .b(new_n170), .c(new_n171), .d(new_n164), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n174), .b(new_n176), .c(new_n159), .d(new_n162), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n04x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nand42aa1n03x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  norp02aa1n04x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand42aa1n06x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanb02aa1n02x5               g087(.a(new_n181), .b(new_n182), .out0(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n179), .c(new_n177), .d(new_n180), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n179), .b(new_n180), .out0(new_n185));
  aoai13aa1n02x5               g090(.a(new_n175), .b(new_n161), .c(new_n129), .d(new_n158), .o1(new_n186));
  norb03aa1n03x5               g091(.a(new_n182), .b(new_n179), .c(new_n181), .out0(new_n187));
  aoai13aa1n02x5               g092(.a(new_n187), .b(new_n185), .c(new_n186), .d(new_n174), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n184), .b(new_n188), .o1(\s[16] ));
  nano23aa1n06x5               g094(.a(new_n144), .b(new_n147), .c(new_n148), .d(new_n145), .out0(new_n190));
  nona22aa1n03x5               g095(.a(new_n175), .b(new_n183), .c(new_n185), .out0(new_n191));
  nano32aa1n03x7               g096(.a(new_n191), .b(new_n190), .c(new_n134), .d(new_n130), .out0(new_n192));
  aoai13aa1n12x5               g097(.a(new_n192), .b(new_n156), .c(new_n155), .d(new_n117), .o1(new_n193));
  nona23aa1n06x5               g098(.a(new_n182), .b(new_n180), .c(new_n179), .d(new_n181), .out0(new_n194));
  nor043aa1n03x5               g099(.a(new_n194), .b(new_n172), .c(new_n165), .o1(new_n195));
  obai22aa1n06x5               g100(.a(new_n182), .b(new_n187), .c(new_n194), .d(new_n174), .out0(new_n196));
  aoi012aa1d18x5               g101(.a(new_n196), .b(new_n161), .c(new_n195), .o1(new_n197));
  xnrc02aa1n12x5               g102(.a(\b[16] ), .b(\a[17] ), .out0(new_n198));
  xobna2aa1n03x5               g103(.a(new_n198), .b(new_n193), .c(new_n197), .out0(\s[17] ));
  inv000aa1d42x5               g104(.a(\a[17] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(\b[16] ), .o1(new_n201));
  nona23aa1n03x5               g106(.a(new_n175), .b(new_n140), .c(new_n194), .d(new_n157), .out0(new_n202));
  aoai13aa1n09x5               g107(.a(new_n197), .b(new_n202), .c(new_n118), .d(new_n128), .o1(new_n203));
  oaoi03aa1n02x5               g108(.a(new_n200), .b(new_n201), .c(new_n203), .o1(new_n204));
  nor002aa1d32x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand42aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nanb02aa1n09x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  nor022aa1n16x5               g112(.a(\b[16] ), .b(\a[17] ), .o1(new_n208));
  norb03aa1n02x5               g113(.a(new_n206), .b(new_n208), .c(new_n205), .out0(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n198), .c(new_n193), .d(new_n197), .o1(new_n210));
  oaib12aa1n02x5               g115(.a(new_n210), .b(new_n204), .c(new_n207), .out0(\s[18] ));
  norp02aa1n02x5               g116(.a(new_n198), .b(new_n207), .o1(new_n212));
  inv000aa1n02x5               g117(.a(new_n212), .o1(new_n213));
  and002aa1n12x5               g118(.a(\b[17] ), .b(\a[18] ), .o(new_n214));
  oab012aa1d24x5               g119(.a(new_n214), .b(new_n208), .c(new_n205), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n213), .c(new_n193), .d(new_n197), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nanp02aa1n12x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nor042aa1n06x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  nand42aa1n16x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nanb02aa1n02x5               g128(.a(new_n222), .b(new_n223), .out0(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n220), .c(new_n217), .d(new_n221), .o1(new_n225));
  nanp02aa1n06x5               g130(.a(new_n203), .b(new_n212), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(new_n220), .b(new_n221), .out0(new_n227));
  norb03aa1n02x5               g132(.a(new_n223), .b(new_n220), .c(new_n222), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n227), .c(new_n226), .d(new_n216), .o1(new_n229));
  nanp02aa1n03x5               g134(.a(new_n225), .b(new_n229), .o1(\s[20] ));
  nano23aa1d15x5               g135(.a(new_n220), .b(new_n222), .c(new_n223), .d(new_n221), .out0(new_n231));
  norb03aa1d15x5               g136(.a(new_n231), .b(new_n198), .c(new_n207), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  nanp02aa1n04x5               g138(.a(new_n231), .b(new_n215), .o1(new_n234));
  oai012aa1n12x5               g139(.a(new_n223), .b(new_n222), .c(new_n220), .o1(new_n235));
  nand02aa1n04x5               g140(.a(new_n234), .b(new_n235), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n233), .c(new_n193), .d(new_n197), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  nand02aa1d16x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n240), .out0(new_n242));
  nor042aa1d18x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nand02aa1n12x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n240), .c(new_n238), .d(new_n242), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n242), .b(new_n236), .c(new_n203), .d(new_n232), .o1(new_n248));
  nona22aa1n03x5               g153(.a(new_n248), .b(new_n246), .c(new_n240), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n247), .b(new_n249), .o1(\s[22] ));
  nano23aa1d15x5               g155(.a(new_n240), .b(new_n243), .c(new_n244), .d(new_n241), .out0(new_n251));
  nona23aa1n02x4               g156(.a(new_n251), .b(new_n231), .c(new_n198), .d(new_n207), .out0(new_n252));
  and002aa1n02x5               g157(.a(\b[21] ), .b(\a[22] ), .o(new_n253));
  oab012aa1n03x5               g158(.a(new_n253), .b(new_n240), .c(new_n243), .out0(new_n254));
  aoi012aa1n03x5               g159(.a(new_n254), .b(new_n236), .c(new_n251), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n252), .c(new_n193), .d(new_n197), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  nand42aa1d28x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  nor002aa1d32x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  nand42aa1d28x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n258), .c(new_n256), .d(new_n260), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n256), .b(new_n260), .o1(new_n266));
  nona22aa1d36x5               g171(.a(new_n262), .b(new_n261), .c(new_n258), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  nand42aa1n02x5               g173(.a(new_n266), .b(new_n268), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(new_n265), .b(new_n269), .o1(\s[24] ));
  nano23aa1n09x5               g175(.a(new_n258), .b(new_n261), .c(new_n262), .d(new_n259), .out0(new_n271));
  nand22aa1n06x5               g176(.a(new_n271), .b(new_n251), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(new_n232), .b(new_n273), .o1(new_n274));
  aoi022aa1n12x5               g179(.a(new_n271), .b(new_n254), .c(new_n262), .d(new_n267), .o1(new_n275));
  aoai13aa1n12x5               g180(.a(new_n275), .b(new_n272), .c(new_n234), .d(new_n235), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n274), .c(new_n193), .d(new_n197), .o1(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  xorc02aa1n06x5               g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  tech160nm_fixnrc02aa1n05x5   g186(.a(\b[25] ), .b(\a[26] ), .out0(new_n282));
  aoai13aa1n03x5               g187(.a(new_n282), .b(new_n280), .c(new_n278), .d(new_n281), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(new_n278), .b(new_n281), .o1(new_n284));
  oabi12aa1n06x5               g189(.a(new_n282), .b(\a[25] ), .c(\b[24] ), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  nanp02aa1n03x5               g191(.a(new_n284), .b(new_n286), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(new_n283), .b(new_n287), .o1(\s[26] ));
  norb02aa1n02x7               g193(.a(new_n281), .b(new_n282), .out0(new_n289));
  nand23aa1n06x5               g194(.a(new_n232), .b(new_n273), .c(new_n289), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(\b[25] ), .b(\a[26] ), .o1(new_n291));
  aoi022aa1n06x5               g196(.a(new_n276), .b(new_n289), .c(new_n291), .d(new_n285), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n290), .c(new_n193), .d(new_n197), .o1(new_n293));
  xorb03aa1n03x5               g198(.a(new_n293), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[27] ), .b(\a[28] ), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n295), .c(new_n293), .d(new_n296), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n290), .o1(new_n299));
  nand22aa1n03x5               g204(.a(new_n276), .b(new_n289), .o1(new_n300));
  oaib12aa1n06x5               g205(.a(new_n300), .b(new_n286), .c(new_n291), .out0(new_n301));
  aoai13aa1n02x7               g206(.a(new_n296), .b(new_n301), .c(new_n203), .d(new_n299), .o1(new_n302));
  nona22aa1n03x5               g207(.a(new_n302), .b(new_n297), .c(new_n295), .out0(new_n303));
  nanp02aa1n03x5               g208(.a(new_n298), .b(new_n303), .o1(\s[28] ));
  tech160nm_fixorc02aa1n03p5x5 g209(.a(\a[29] ), .b(\b[28] ), .out0(new_n305));
  inv000aa1d42x5               g210(.a(new_n305), .o1(new_n306));
  norb02aa1n02x5               g211(.a(new_n296), .b(new_n297), .out0(new_n307));
  inv000aa1d42x5               g212(.a(\a[28] ), .o1(new_n308));
  inv000aa1d42x5               g213(.a(\b[27] ), .o1(new_n309));
  oaoi03aa1n06x5               g214(.a(new_n308), .b(new_n309), .c(new_n295), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n310), .o1(new_n311));
  aoai13aa1n02x7               g216(.a(new_n306), .b(new_n311), .c(new_n293), .d(new_n307), .o1(new_n312));
  aoai13aa1n02x7               g217(.a(new_n307), .b(new_n301), .c(new_n203), .d(new_n299), .o1(new_n313));
  nona22aa1n03x5               g218(.a(new_n313), .b(new_n311), .c(new_n306), .out0(new_n314));
  nanp02aa1n03x5               g219(.a(new_n312), .b(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g221(.a(new_n297), .b(new_n296), .c(new_n305), .out0(new_n317));
  oaoi03aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .o1(new_n318));
  tech160nm_fixorc02aa1n03p5x5 g223(.a(\a[30] ), .b(\b[29] ), .out0(new_n319));
  inv000aa1d42x5               g224(.a(new_n319), .o1(new_n320));
  aoai13aa1n02x7               g225(.a(new_n320), .b(new_n318), .c(new_n293), .d(new_n317), .o1(new_n321));
  aoai13aa1n02x7               g226(.a(new_n317), .b(new_n301), .c(new_n203), .d(new_n299), .o1(new_n322));
  nona22aa1n03x5               g227(.a(new_n322), .b(new_n318), .c(new_n320), .out0(new_n323));
  nanp02aa1n03x5               g228(.a(new_n321), .b(new_n323), .o1(\s[30] ));
  xnrc02aa1n02x5               g229(.a(\b[30] ), .b(\a[31] ), .out0(new_n325));
  nano23aa1n02x4               g230(.a(new_n320), .b(new_n297), .c(new_n296), .d(new_n305), .out0(new_n326));
  and002aa1n02x5               g231(.a(\b[29] ), .b(\a[30] ), .o(new_n327));
  oab012aa1n02x4               g232(.a(new_n327), .b(new_n318), .c(new_n320), .out0(new_n328));
  aoai13aa1n02x7               g233(.a(new_n325), .b(new_n328), .c(new_n293), .d(new_n326), .o1(new_n329));
  aoai13aa1n02x7               g234(.a(new_n326), .b(new_n301), .c(new_n203), .d(new_n299), .o1(new_n330));
  nona22aa1n03x5               g235(.a(new_n330), .b(new_n328), .c(new_n325), .out0(new_n331));
  nanp02aa1n03x5               g236(.a(new_n329), .b(new_n331), .o1(\s[31] ));
  xnbna2aa1n03x5               g237(.a(new_n100), .b(new_n103), .c(new_n106), .out0(\s[3] ));
  orn002aa1n02x5               g238(.a(new_n100), .b(new_n104), .o(new_n334));
  xobna2aa1n03x5               g239(.a(new_n101), .b(new_n334), .c(new_n106), .out0(\s[4] ));
  xorb03aa1n02x5               g240(.a(new_n155), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g241(.a(new_n119), .b(new_n120), .c(new_n155), .o1(new_n337));
  oabi12aa1n02x5               g242(.a(new_n113), .b(new_n105), .c(new_n107), .out0(new_n338));
  nanp02aa1n02x5               g243(.a(new_n338), .b(new_n122), .o1(new_n339));
  oaib12aa1n02x5               g244(.a(new_n339), .b(new_n337), .c(new_n116), .out0(\s[6] ));
  norb02aa1n02x5               g245(.a(new_n111), .b(new_n110), .out0(new_n341));
  xobna2aa1n03x5               g246(.a(new_n341), .b(new_n339), .c(new_n115), .out0(\s[7] ));
  nanp03aa1n02x5               g247(.a(new_n339), .b(new_n341), .c(new_n115), .o1(new_n343));
  xobna2aa1n03x5               g248(.a(new_n124), .b(new_n343), .c(new_n125), .out0(\s[8] ));
  xnbna2aa1n03x5               g249(.a(new_n130), .b(new_n118), .c(new_n128), .out0(\s[9] ));
endmodule


