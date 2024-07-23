// Benchmark "adder" written by ABC on Wed Jul 17 14:06:00 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n328, new_n330, new_n331, new_n333,
    new_n334, new_n335, new_n336, new_n338, new_n339, new_n342, new_n343,
    new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n16x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d16x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(new_n98), .o1(new_n99));
  nor002aa1n16x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  orn002aa1n12x5               g006(.a(\a[2] ), .b(\b[1] ), .o(new_n102));
  nanp02aa1n06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n06x5               g008(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(new_n104));
  inv040aa1d32x5               g009(.a(\a[3] ), .o1(new_n105));
  inv030aa1d32x5               g010(.a(\b[2] ), .o1(new_n106));
  nand22aa1n06x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanp02aa1n03x5               g013(.a(new_n107), .b(new_n108), .o1(new_n109));
  oa0022aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  aoai13aa1n06x5               g015(.a(new_n110), .b(new_n109), .c(new_n104), .d(new_n102), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nor022aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  aoi012aa1n02x5               g018(.a(new_n113), .b(\a[6] ), .c(\b[5] ), .o1(new_n114));
  nor002aa1n03x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  aoi012aa1n02x5               g020(.a(new_n115), .b(\a[4] ), .c(\b[3] ), .o1(new_n116));
  nand22aa1n04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor002aa1d32x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nand02aa1d04x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nor002aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  nona23aa1n02x4               g025(.a(new_n119), .b(new_n117), .c(new_n120), .d(new_n118), .out0(new_n121));
  nano32aa1n03x7               g026(.a(new_n121), .b(new_n116), .c(new_n114), .d(new_n112), .out0(new_n122));
  nanp02aa1n12x5               g027(.a(new_n122), .b(new_n111), .o1(new_n123));
  nand02aa1n03x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  nano22aa1n03x7               g029(.a(new_n118), .b(new_n124), .c(new_n119), .out0(new_n125));
  oai022aa1n02x7               g030(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n126));
  norb02aa1n06x4               g031(.a(new_n117), .b(new_n115), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n118), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(\a[8] ), .b(\b[7] ), .c(new_n128), .o1(new_n129));
  aoi013aa1n06x4               g034(.a(new_n129), .b(new_n125), .c(new_n127), .d(new_n126), .o1(new_n130));
  nanp02aa1n06x5               g035(.a(new_n123), .b(new_n130), .o1(new_n131));
  nanp02aa1n03x5               g036(.a(new_n131), .b(new_n101), .o1(new_n132));
  obai22aa1n02x7               g037(.a(new_n132), .b(new_n100), .c(new_n97), .d(new_n99), .out0(new_n133));
  nona32aa1n06x5               g038(.a(new_n132), .b(new_n100), .c(new_n99), .d(new_n97), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(new_n133), .b(new_n134), .o1(\s[10] ));
  nand02aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor002aa1d24x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n137), .b(\a[10] ), .c(\b[9] ), .o1(new_n138));
  inv000aa1n04x5               g043(.a(new_n137), .o1(new_n139));
  aoi022aa1n02x5               g044(.a(new_n134), .b(new_n98), .c(new_n139), .d(new_n136), .o1(new_n140));
  aoi013aa1n02x4               g045(.a(new_n140), .b(new_n138), .c(new_n136), .d(new_n134), .o1(\s[11] ));
  nand23aa1n03x5               g046(.a(new_n134), .b(new_n136), .c(new_n138), .o1(new_n142));
  nor042aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand22aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n03x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  norp02aa1n02x5               g050(.a(new_n145), .b(new_n137), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n142), .b(new_n139), .o1(new_n147));
  aoi022aa1n02x5               g052(.a(new_n147), .b(new_n145), .c(new_n142), .d(new_n146), .o1(\s[12] ));
  nona23aa1n02x4               g053(.a(new_n101), .b(new_n98), .c(new_n97), .d(new_n100), .out0(new_n149));
  nano32aa1n03x7               g054(.a(new_n149), .b(new_n145), .c(new_n139), .d(new_n136), .out0(new_n150));
  nanb03aa1n06x5               g055(.a(new_n143), .b(new_n144), .c(new_n136), .out0(new_n151));
  oai112aa1n03x5               g056(.a(new_n139), .b(new_n98), .c(new_n100), .d(new_n97), .o1(new_n152));
  tech160nm_fiaoi012aa1n03p5x5 g057(.a(new_n143), .b(new_n137), .c(new_n144), .o1(new_n153));
  tech160nm_fioai012aa1n05x5   g058(.a(new_n153), .b(new_n152), .c(new_n151), .o1(new_n154));
  nor002aa1d32x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n04x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n03x5               g063(.a(new_n158), .b(new_n154), .c(new_n131), .d(new_n150), .o1(new_n159));
  oai112aa1n02x5               g064(.a(new_n153), .b(new_n157), .c(new_n152), .d(new_n151), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n131), .c(new_n150), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n159), .b(new_n161), .out0(\s[13] ));
  inv000aa1d42x5               g067(.a(new_n155), .o1(new_n163));
  norp02aa1n12x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand22aa1n12x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n159), .c(new_n163), .out0(\s[14] ));
  nona23aa1d18x5               g072(.a(new_n165), .b(new_n156), .c(new_n155), .d(new_n164), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n154), .c(new_n131), .d(new_n150), .o1(new_n170));
  tech160nm_fiaoi012aa1n03p5x5 g075(.a(new_n164), .b(new_n155), .c(new_n165), .o1(new_n171));
  nor042aa1d18x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nanb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n170), .c(new_n171), .out0(\s[15] ));
  aob012aa1n03x5               g081(.a(new_n175), .b(new_n170), .c(new_n171), .out0(new_n177));
  nor042aa1n03x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand22aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(new_n180));
  aoib12aa1n02x5               g085(.a(new_n172), .b(new_n179), .c(new_n178), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n172), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n174), .c(new_n170), .d(new_n171), .o1(new_n183));
  aboi22aa1n03x5               g088(.a(new_n180), .b(new_n183), .c(new_n177), .d(new_n181), .out0(\s[16] ));
  nona23aa1n09x5               g089(.a(new_n179), .b(new_n173), .c(new_n172), .d(new_n178), .out0(new_n185));
  nor042aa1n04x5               g090(.a(new_n185), .b(new_n168), .o1(new_n186));
  nand22aa1n03x5               g091(.a(new_n150), .b(new_n186), .o1(new_n187));
  aoi012aa1n02x5               g092(.a(new_n187), .b(new_n123), .c(new_n130), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n178), .b(new_n172), .c(new_n179), .o1(new_n189));
  tech160nm_fioai012aa1n02p5x5 g094(.a(new_n189), .b(new_n185), .c(new_n171), .o1(new_n190));
  aoi012aa1n06x5               g095(.a(new_n190), .b(new_n154), .c(new_n186), .o1(new_n191));
  aoai13aa1n12x5               g096(.a(new_n191), .b(new_n187), .c(new_n123), .d(new_n130), .o1(new_n192));
  nor042aa1d18x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  nand42aa1n02x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  oaib12aa1n02x5               g099(.a(new_n192), .b(new_n193), .c(new_n194), .out0(new_n195));
  oai022aa1n02x5               g100(.a(\a[16] ), .b(\b[15] ), .c(\b[16] ), .d(\a[17] ), .o1(new_n196));
  aoi122aa1n02x5               g101(.a(new_n196), .b(\b[16] ), .c(\a[17] ), .d(new_n172), .e(new_n179), .o1(new_n197));
  oai012aa1n02x5               g102(.a(new_n197), .b(new_n185), .c(new_n171), .o1(new_n198));
  aoi012aa1n02x5               g103(.a(new_n198), .b(new_n154), .c(new_n186), .o1(new_n199));
  oaib12aa1n02x5               g104(.a(new_n195), .b(new_n188), .c(new_n199), .out0(\s[17] ));
  inv000aa1n09x5               g105(.a(new_n193), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(new_n192), .b(new_n194), .o1(new_n202));
  xorc02aa1n12x5               g107(.a(\a[18] ), .b(\b[17] ), .out0(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n202), .c(new_n201), .out0(\s[18] ));
  and003aa1n12x5               g109(.a(new_n203), .b(new_n194), .c(new_n201), .o(new_n205));
  oaoi03aa1n09x5               g110(.a(\a[18] ), .b(\b[17] ), .c(new_n201), .o1(new_n206));
  tech160nm_fixorc02aa1n05x5   g111(.a(\a[19] ), .b(\b[18] ), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n206), .c(new_n192), .d(new_n205), .o1(new_n208));
  aoi112aa1n02x5               g113(.a(new_n207), .b(new_n206), .c(new_n192), .d(new_n205), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n12x5               g116(.a(\a[20] ), .b(\b[19] ), .out0(new_n212));
  norp02aa1n02x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norp02aa1n02x5               g118(.a(new_n212), .b(new_n213), .o1(new_n214));
  inv000aa1d42x5               g119(.a(\a[19] ), .o1(new_n215));
  oaib12aa1n03x5               g120(.a(new_n208), .b(\b[18] ), .c(new_n215), .out0(new_n216));
  aoi022aa1n02x5               g121(.a(new_n216), .b(new_n212), .c(new_n208), .d(new_n214), .o1(\s[20] ));
  inv000aa1d42x5               g122(.a(\a[20] ), .o1(new_n218));
  xroi22aa1d04x5               g123(.a(new_n215), .b(\b[18] ), .c(new_n218), .d(\b[19] ), .out0(new_n219));
  nand02aa1d04x5               g124(.a(new_n205), .b(new_n219), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  nanp03aa1n06x5               g126(.a(new_n206), .b(new_n207), .c(new_n212), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[19] ), .o1(new_n223));
  oaoi03aa1n02x5               g128(.a(new_n218), .b(new_n223), .c(new_n213), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(new_n222), .b(new_n224), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .out0(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n225), .c(new_n192), .d(new_n221), .o1(new_n227));
  nano22aa1n02x4               g132(.a(new_n226), .b(new_n222), .c(new_n224), .out0(new_n228));
  aobi12aa1n02x5               g133(.a(new_n228), .b(new_n192), .c(new_n221), .out0(new_n229));
  norb02aa1n02x5               g134(.a(new_n227), .b(new_n229), .out0(\s[21] ));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(\a[21] ), .o1(new_n232));
  aoib12aa1n02x5               g137(.a(new_n231), .b(new_n232), .c(\b[20] ), .out0(new_n233));
  oaib12aa1n03x5               g138(.a(new_n227), .b(\b[20] ), .c(new_n232), .out0(new_n234));
  aoi022aa1n02x7               g139(.a(new_n234), .b(new_n231), .c(new_n227), .d(new_n233), .o1(\s[22] ));
  inv000aa1d42x5               g140(.a(\a[22] ), .o1(new_n236));
  xroi22aa1d06x4               g141(.a(new_n232), .b(\b[20] ), .c(new_n236), .d(\b[21] ), .out0(new_n237));
  and003aa1n02x5               g142(.a(new_n205), .b(new_n237), .c(new_n219), .o(new_n238));
  inv020aa1n02x5               g143(.a(new_n237), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(\b[20] ), .b(\a[21] ), .c(\a[22] ), .d(\b[21] ), .o1(new_n240));
  aoib12aa1n02x5               g145(.a(new_n240), .b(new_n236), .c(\b[21] ), .out0(new_n241));
  aoai13aa1n04x5               g146(.a(new_n241), .b(new_n239), .c(new_n222), .d(new_n224), .o1(new_n242));
  tech160nm_fixorc02aa1n02p5x5 g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n242), .c(new_n192), .d(new_n238), .o1(new_n244));
  nanb02aa1n02x5               g149(.a(new_n243), .b(new_n241), .out0(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n225), .c(new_n237), .o1(new_n246));
  aobi12aa1n02x5               g151(.a(new_n246), .b(new_n192), .c(new_n238), .out0(new_n247));
  norb02aa1n02x5               g152(.a(new_n244), .b(new_n247), .out0(\s[23] ));
  xorc02aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  oai012aa1n03x5               g156(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .o1(new_n252));
  aoi022aa1n02x7               g157(.a(new_n252), .b(new_n249), .c(new_n244), .d(new_n251), .o1(\s[24] ));
  and002aa1n02x5               g158(.a(new_n249), .b(new_n243), .o(new_n254));
  nano22aa1n02x4               g159(.a(new_n220), .b(new_n237), .c(new_n254), .out0(new_n255));
  nanp02aa1n03x5               g160(.a(new_n192), .b(new_n255), .o1(new_n256));
  aob012aa1n02x5               g161(.a(new_n250), .b(\b[23] ), .c(\a[24] ), .out0(new_n257));
  oai012aa1n02x5               g162(.a(new_n257), .b(\b[23] ), .c(\a[24] ), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n242), .c(new_n254), .o1(new_n259));
  nand22aa1n03x5               g164(.a(new_n256), .b(new_n259), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n261), .b(new_n258), .c(new_n242), .d(new_n254), .o1(new_n262));
  aoi022aa1n02x5               g167(.a(new_n260), .b(new_n261), .c(new_n256), .d(new_n262), .o1(\s[25] ));
  nanp02aa1n03x5               g168(.a(new_n260), .b(new_n261), .o1(new_n264));
  xorc02aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .out0(new_n265));
  nor042aa1n09x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n265), .b(new_n266), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n266), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n261), .o1(new_n269));
  aoai13aa1n02x5               g174(.a(new_n268), .b(new_n269), .c(new_n256), .d(new_n259), .o1(new_n270));
  aoi022aa1n03x5               g175(.a(new_n270), .b(new_n265), .c(new_n264), .d(new_n267), .o1(\s[26] ));
  and002aa1n02x5               g176(.a(new_n265), .b(new_n261), .o(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n258), .c(new_n242), .d(new_n254), .o1(new_n273));
  nano32aa1n03x7               g178(.a(new_n220), .b(new_n272), .c(new_n237), .d(new_n254), .out0(new_n274));
  norp02aa1n02x5               g179(.a(\b[25] ), .b(\a[26] ), .o1(new_n275));
  aob012aa1n02x5               g180(.a(new_n266), .b(\b[25] ), .c(\a[26] ), .out0(new_n276));
  norb02aa1n02x5               g181(.a(new_n276), .b(new_n275), .out0(new_n277));
  inv000aa1n02x5               g182(.a(new_n277), .o1(new_n278));
  tech160nm_fiaoi012aa1n05x5   g183(.a(new_n278), .b(new_n192), .c(new_n274), .o1(new_n279));
  nor042aa1n03x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  and002aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o(new_n281));
  norp02aa1n02x5               g186(.a(new_n281), .b(new_n280), .o1(new_n282));
  nand02aa1d06x5               g187(.a(new_n192), .b(new_n274), .o1(new_n283));
  inv000aa1n06x5               g188(.a(new_n280), .o1(new_n284));
  nano23aa1n02x4               g189(.a(new_n281), .b(new_n275), .c(new_n276), .d(new_n284), .out0(new_n285));
  nanp03aa1n02x5               g190(.a(new_n273), .b(new_n283), .c(new_n285), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n282), .c(new_n279), .d(new_n273), .o1(\s[27] ));
  inv000aa1d42x5               g192(.a(\b[26] ), .o1(new_n288));
  nand23aa1n03x5               g193(.a(new_n273), .b(new_n283), .c(new_n277), .o1(new_n289));
  oaib12aa1n03x5               g194(.a(new_n289), .b(new_n288), .c(\a[27] ), .out0(new_n290));
  aoai13aa1n03x5               g195(.a(new_n284), .b(new_n281), .c(new_n279), .d(new_n273), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .out0(new_n292));
  norp02aa1n02x5               g197(.a(new_n292), .b(new_n280), .o1(new_n293));
  aoi022aa1n03x5               g198(.a(new_n291), .b(new_n292), .c(new_n290), .d(new_n293), .o1(\s[28] ));
  inv000aa1d42x5               g199(.a(\a[28] ), .o1(new_n295));
  xroi22aa1d04x5               g200(.a(\a[27] ), .b(new_n288), .c(new_n295), .d(\b[27] ), .out0(new_n296));
  nand02aa1n02x5               g201(.a(new_n289), .b(new_n296), .o1(new_n297));
  inv000aa1n03x5               g202(.a(new_n296), .o1(new_n298));
  oaoi03aa1n12x5               g203(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n299), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n298), .c(new_n279), .d(new_n273), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n299), .b(new_n302), .o1(new_n303));
  aoi022aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n297), .d(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g210(.a(new_n292), .b(new_n302), .c(new_n282), .o(new_n306));
  nanp02aa1n03x5               g211(.a(new_n289), .b(new_n306), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n306), .o1(new_n308));
  inv000aa1d42x5               g213(.a(\b[28] ), .o1(new_n309));
  oaib12aa1n09x5               g214(.a(new_n299), .b(new_n309), .c(\a[29] ), .out0(new_n310));
  oa0012aa1n02x5               g215(.a(new_n310), .b(\b[28] ), .c(\a[29] ), .o(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n279), .d(new_n273), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .out0(new_n313));
  oabi12aa1n02x5               g218(.a(new_n313), .b(\a[29] ), .c(\b[28] ), .out0(new_n314));
  norb02aa1n02x5               g219(.a(new_n310), .b(new_n314), .out0(new_n315));
  aoi022aa1n03x5               g220(.a(new_n312), .b(new_n313), .c(new_n307), .d(new_n315), .o1(\s[30] ));
  nano22aa1n02x4               g221(.a(new_n298), .b(new_n302), .c(new_n313), .out0(new_n317));
  nand02aa1n02x5               g222(.a(new_n289), .b(new_n317), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[31] ), .b(\b[30] ), .out0(new_n319));
  oai122aa1n02x7               g224(.a(new_n310), .b(\a[30] ), .c(\b[29] ), .d(\a[29] ), .e(\b[28] ), .o1(new_n320));
  aob012aa1n03x5               g225(.a(new_n320), .b(\b[29] ), .c(\a[30] ), .out0(new_n321));
  norb02aa1n02x5               g226(.a(new_n321), .b(new_n319), .out0(new_n322));
  inv000aa1n02x5               g227(.a(new_n317), .o1(new_n323));
  aoai13aa1n03x5               g228(.a(new_n321), .b(new_n323), .c(new_n279), .d(new_n273), .o1(new_n324));
  aoi022aa1n03x5               g229(.a(new_n324), .b(new_n319), .c(new_n318), .d(new_n322), .o1(\s[31] ));
  xobna2aa1n03x5               g230(.a(new_n109), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  tech160nm_fiao0012aa1n02p5x5 g231(.a(new_n109), .b(new_n102), .c(new_n104), .o(new_n327));
  xorc02aa1n02x5               g232(.a(\a[4] ), .b(\b[3] ), .out0(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n328), .b(new_n327), .c(new_n107), .out0(\s[4] ));
  nanp02aa1n02x5               g234(.a(\b[3] ), .b(\a[4] ), .o1(new_n330));
  nanb02aa1n02x5               g235(.a(new_n113), .b(new_n112), .out0(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n331), .b(new_n111), .c(new_n330), .out0(\s[5] ));
  norb02aa1n02x5               g237(.a(new_n124), .b(new_n120), .out0(new_n333));
  aoi013aa1n02x4               g238(.a(new_n113), .b(new_n111), .c(new_n112), .d(new_n330), .o1(new_n334));
  nanp03aa1n02x5               g239(.a(new_n111), .b(new_n112), .c(new_n330), .o1(new_n335));
  oai112aa1n02x5               g240(.a(new_n335), .b(new_n333), .c(\b[4] ), .d(\a[5] ), .o1(new_n336));
  oai012aa1n02x5               g241(.a(new_n336), .b(new_n334), .c(new_n333), .o1(\s[6] ));
  nanp02aa1n02x5               g242(.a(new_n336), .b(new_n125), .o1(new_n338));
  aoi022aa1n02x5               g243(.a(new_n336), .b(new_n124), .c(new_n128), .d(new_n119), .o1(new_n339));
  norb02aa1n02x5               g244(.a(new_n338), .b(new_n339), .out0(\s[7] ));
  xnbna2aa1n03x5               g245(.a(new_n127), .b(new_n338), .c(new_n128), .out0(\s[8] ));
  nanb02aa1n02x5               g246(.a(new_n100), .b(new_n101), .out0(new_n342));
  inv000aa1d42x5               g247(.a(new_n342), .o1(new_n343));
  aoi113aa1n02x5               g248(.a(new_n129), .b(new_n343), .c(new_n125), .d(new_n127), .e(new_n126), .o1(new_n344));
  aoi022aa1n02x5               g249(.a(new_n131), .b(new_n343), .c(new_n344), .d(new_n123), .o1(\s[9] ));
endmodule


