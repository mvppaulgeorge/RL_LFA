// Benchmark "adder" written by ABC on Thu Jul 18 15:24:59 2024

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
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n345, new_n347, new_n348, new_n350, new_n351, new_n352, new_n355,
    new_n356, new_n359, new_n360;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n12x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nona22aa1n09x5               g005(.a(new_n99), .b(new_n98), .c(new_n100), .out0(new_n101));
  oai012aa1n12x5               g006(.a(new_n99), .b(\b[3] ), .c(\a[4] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand02aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor042aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano23aa1n09x5               g010(.a(new_n102), .b(new_n105), .c(new_n103), .d(new_n104), .out0(new_n106));
  nanp02aa1n04x5               g011(.a(new_n106), .b(new_n101), .o1(new_n107));
  nor002aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  aoi012aa1n09x5               g013(.a(new_n108), .b(new_n105), .c(new_n104), .o1(new_n109));
  nand42aa1n06x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor022aa1n06x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nanb02aa1n12x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nor002aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n08x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  norb02aa1n03x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  xorc02aa1n12x5               g020(.a(\a[8] ), .b(\b[7] ), .out0(new_n116));
  nor002aa1d32x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanp02aa1n04x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanb02aa1n03x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  nona23aa1d18x5               g024(.a(new_n116), .b(new_n115), .c(new_n119), .d(new_n112), .out0(new_n120));
  nano22aa1n06x5               g025(.a(new_n117), .b(new_n110), .c(new_n118), .out0(new_n121));
  oai022aa1n02x7               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  inv000aa1n02x5               g027(.a(new_n117), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  aoi013aa1n06x4               g029(.a(new_n124), .b(new_n121), .c(new_n116), .d(new_n122), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n120), .c(new_n107), .d(new_n109), .o1(new_n126));
  nor002aa1n03x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n127), .b(new_n128), .out0(new_n129));
  inv000aa1d42x5               g034(.a(new_n129), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n126), .b(new_n130), .o1(new_n131));
  nor002aa1n03x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand22aa1n03x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(new_n134));
  xobna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n97), .out0(\s[10] ));
  nona22aa1n03x5               g040(.a(new_n126), .b(new_n129), .c(new_n134), .out0(new_n136));
  tech160nm_fiaoi012aa1n04x5   g041(.a(new_n132), .b(new_n127), .c(new_n133), .o1(new_n137));
  nor002aa1n16x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand42aa1n08x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n03x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n136), .c(new_n137), .out0(\s[11] ));
  aob012aa1n02x5               g046(.a(new_n140), .b(new_n136), .c(new_n137), .out0(new_n142));
  nor042aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n16x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  inv000aa1d42x5               g050(.a(\a[11] ), .o1(new_n146));
  inv000aa1d42x5               g051(.a(\b[10] ), .o1(new_n147));
  aboi22aa1n03x5               g052(.a(new_n143), .b(new_n144), .c(new_n146), .d(new_n147), .out0(new_n148));
  inv020aa1n04x5               g053(.a(new_n138), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n142), .b(new_n149), .o1(new_n150));
  aoi022aa1n02x5               g055(.a(new_n150), .b(new_n145), .c(new_n142), .d(new_n148), .o1(\s[12] ));
  nona23aa1n03x5               g056(.a(new_n133), .b(new_n128), .c(new_n127), .d(new_n132), .out0(new_n152));
  nano22aa1n03x7               g057(.a(new_n152), .b(new_n140), .c(new_n145), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n126), .b(new_n153), .o1(new_n154));
  inv030aa1n03x5               g059(.a(new_n137), .o1(new_n155));
  nano23aa1n03x5               g060(.a(new_n138), .b(new_n143), .c(new_n144), .d(new_n139), .out0(new_n156));
  nanp02aa1n02x5               g061(.a(new_n156), .b(new_n155), .o1(new_n157));
  oaoi03aa1n12x5               g062(.a(\a[12] ), .b(\b[11] ), .c(new_n149), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n157), .b(new_n159), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n160), .b(new_n154), .out0(new_n161));
  tech160nm_fixnrc02aa1n04x5   g066(.a(\b[12] ), .b(\a[13] ), .out0(new_n162));
  and003aa1n02x5               g067(.a(new_n157), .b(new_n162), .c(new_n159), .o(new_n163));
  aboi22aa1n03x5               g068(.a(new_n162), .b(new_n161), .c(new_n163), .d(new_n154), .out0(\s[13] ));
  nanb02aa1n03x5               g069(.a(new_n162), .b(new_n161), .out0(new_n165));
  tech160nm_fixnrc02aa1n04x5   g070(.a(\b[13] ), .b(\a[14] ), .out0(new_n166));
  norp02aa1n02x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  oai012aa1n02x5               g073(.a(new_n165), .b(\b[12] ), .c(\a[13] ), .o1(new_n169));
  aboi22aa1n03x5               g074(.a(new_n166), .b(new_n169), .c(new_n165), .d(new_n168), .out0(\s[14] ));
  nor002aa1n02x5               g075(.a(new_n166), .b(new_n162), .o1(new_n171));
  aoai13aa1n03x5               g076(.a(new_n171), .b(new_n160), .c(new_n126), .d(new_n153), .o1(new_n172));
  norp02aa1n02x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  aoi012aa1n02x5               g079(.a(new_n173), .b(new_n167), .c(new_n174), .o1(new_n175));
  nor002aa1d32x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanp02aa1n04x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n06x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n172), .c(new_n175), .out0(\s[15] ));
  aob012aa1n03x5               g084(.a(new_n178), .b(new_n172), .c(new_n175), .out0(new_n180));
  nor022aa1n12x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nanp02aa1n06x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  aoib12aa1n02x5               g088(.a(new_n176), .b(new_n182), .c(new_n181), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n176), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n178), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n172), .d(new_n175), .o1(new_n187));
  aoi022aa1n02x7               g092(.a(new_n187), .b(new_n183), .c(new_n180), .d(new_n184), .o1(\s[16] ));
  nona23aa1d16x5               g093(.a(new_n182), .b(new_n177), .c(new_n176), .d(new_n181), .out0(new_n189));
  nona32aa1n09x5               g094(.a(new_n153), .b(new_n189), .c(new_n166), .d(new_n162), .out0(new_n190));
  inv040aa1n02x5               g095(.a(new_n190), .o1(new_n191));
  nand02aa1d06x5               g096(.a(new_n126), .b(new_n191), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n189), .o1(new_n193));
  aoai13aa1n04x5               g098(.a(new_n171), .b(new_n158), .c(new_n156), .d(new_n155), .o1(new_n194));
  nand42aa1n02x5               g099(.a(new_n194), .b(new_n175), .o1(new_n195));
  nand22aa1n04x5               g100(.a(new_n195), .b(new_n193), .o1(new_n196));
  oaih12aa1n06x5               g101(.a(new_n182), .b(new_n181), .c(new_n176), .o1(new_n197));
  nand23aa1d12x5               g102(.a(new_n192), .b(new_n196), .c(new_n197), .o1(new_n198));
  tech160nm_fixorc02aa1n05x5   g103(.a(\a[17] ), .b(\b[16] ), .out0(new_n199));
  nano22aa1n02x4               g104(.a(new_n199), .b(new_n196), .c(new_n197), .out0(new_n200));
  aoi022aa1n02x5               g105(.a(new_n200), .b(new_n192), .c(new_n198), .d(new_n199), .o1(\s[17] ));
  inv040aa1d32x5               g106(.a(\a[17] ), .o1(new_n202));
  inv040aa1d28x5               g107(.a(\b[16] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(new_n203), .b(new_n202), .o1(new_n204));
  inv000aa1n02x5               g109(.a(new_n109), .o1(new_n205));
  tech160nm_fiaoi012aa1n04x5   g110(.a(new_n205), .b(new_n106), .c(new_n101), .o1(new_n206));
  oaoi13aa1n12x5               g111(.a(new_n190), .b(new_n125), .c(new_n206), .d(new_n120), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n197), .b(new_n189), .c(new_n194), .d(new_n175), .o1(new_n208));
  oaih12aa1n02x5               g113(.a(new_n199), .b(new_n208), .c(new_n207), .o1(new_n209));
  nor042aa1n02x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  nand02aa1n06x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  norb02aa1n03x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n209), .c(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g118(.a(new_n199), .b(new_n212), .o(new_n214));
  oaih12aa1n02x5               g119(.a(new_n214), .b(new_n208), .c(new_n207), .o1(new_n215));
  aoi013aa1n09x5               g120(.a(new_n210), .b(new_n211), .c(new_n202), .d(new_n203), .o1(new_n216));
  nor002aa1d32x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nand42aa1n20x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n215), .c(new_n216), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n03x5               g126(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n219), .b(new_n222), .c(new_n198), .d(new_n214), .o1(new_n223));
  nor002aa1d32x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nand02aa1n06x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  inv000aa1d42x5               g131(.a(\a[19] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\b[18] ), .o1(new_n228));
  aboi22aa1n03x5               g133(.a(new_n224), .b(new_n225), .c(new_n227), .d(new_n228), .out0(new_n229));
  inv040aa1n08x5               g134(.a(new_n217), .o1(new_n230));
  inv000aa1n02x5               g135(.a(new_n219), .o1(new_n231));
  aoai13aa1n02x7               g136(.a(new_n230), .b(new_n231), .c(new_n215), .d(new_n216), .o1(new_n232));
  aoi022aa1n03x5               g137(.a(new_n232), .b(new_n226), .c(new_n223), .d(new_n229), .o1(\s[20] ));
  nona23aa1d24x5               g138(.a(new_n225), .b(new_n218), .c(new_n217), .d(new_n224), .out0(new_n234));
  nano22aa1n12x5               g139(.a(new_n234), .b(new_n199), .c(new_n212), .out0(new_n235));
  oaih12aa1n02x5               g140(.a(new_n235), .b(new_n208), .c(new_n207), .o1(new_n236));
  oaoi03aa1n12x5               g141(.a(\a[20] ), .b(\b[19] ), .c(new_n230), .o1(new_n237));
  inv040aa1n02x5               g142(.a(new_n237), .o1(new_n238));
  oai012aa1n18x5               g143(.a(new_n238), .b(new_n234), .c(new_n216), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n239), .c(new_n198), .d(new_n235), .o1(new_n242));
  nano23aa1n06x5               g147(.a(new_n217), .b(new_n224), .c(new_n225), .d(new_n218), .out0(new_n243));
  aoi112aa1n02x5               g148(.a(new_n237), .b(new_n241), .c(new_n243), .d(new_n222), .o1(new_n244));
  aobi12aa1n03x7               g149(.a(new_n242), .b(new_n244), .c(new_n236), .out0(\s[21] ));
  xnrc02aa1n12x5               g150(.a(\b[21] ), .b(\a[22] ), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  nor042aa1n06x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n246), .b(new_n248), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n239), .o1(new_n250));
  inv000aa1n06x5               g155(.a(new_n248), .o1(new_n251));
  aoai13aa1n02x7               g156(.a(new_n251), .b(new_n240), .c(new_n236), .d(new_n250), .o1(new_n252));
  aoi022aa1n03x5               g157(.a(new_n252), .b(new_n247), .c(new_n242), .d(new_n249), .o1(\s[22] ));
  nor042aa1n06x5               g158(.a(new_n246), .b(new_n240), .o1(new_n254));
  nano32aa1n02x4               g159(.a(new_n234), .b(new_n254), .c(new_n199), .d(new_n212), .out0(new_n255));
  oaih12aa1n02x5               g160(.a(new_n255), .b(new_n208), .c(new_n207), .o1(new_n256));
  oaoi03aa1n09x5               g161(.a(\a[22] ), .b(\b[21] ), .c(new_n251), .o1(new_n257));
  tech160nm_fiaoi012aa1n03p5x5 g162(.a(new_n257), .b(new_n239), .c(new_n254), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n198), .d(new_n255), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(new_n260), .b(new_n257), .c(new_n239), .d(new_n254), .o1(new_n262));
  aobi12aa1n02x7               g167(.a(new_n261), .b(new_n262), .c(new_n256), .out0(\s[23] ));
  tech160nm_fixorc02aa1n02p5x5 g168(.a(\a[24] ), .b(\b[23] ), .out0(new_n264));
  nor042aa1n09x5               g169(.a(\b[22] ), .b(\a[23] ), .o1(new_n265));
  norp02aa1n02x5               g170(.a(new_n264), .b(new_n265), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n265), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n260), .o1(new_n268));
  aoai13aa1n02x7               g173(.a(new_n267), .b(new_n268), .c(new_n256), .d(new_n258), .o1(new_n269));
  aoi022aa1n02x7               g174(.a(new_n269), .b(new_n264), .c(new_n261), .d(new_n266), .o1(\s[24] ));
  inv000aa1n02x5               g175(.a(new_n235), .o1(new_n271));
  and002aa1n06x5               g176(.a(new_n264), .b(new_n260), .o(new_n272));
  nano22aa1n02x4               g177(.a(new_n271), .b(new_n272), .c(new_n254), .out0(new_n273));
  oaih12aa1n02x5               g178(.a(new_n273), .b(new_n208), .c(new_n207), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n254), .b(new_n237), .c(new_n243), .d(new_n222), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n257), .o1(new_n276));
  inv020aa1n02x5               g181(.a(new_n272), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[24] ), .b(\b[23] ), .c(new_n267), .carry(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n277), .c(new_n275), .d(new_n276), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n279), .c(new_n198), .d(new_n273), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n272), .b(new_n257), .c(new_n239), .d(new_n254), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n280), .o1(new_n283));
  and003aa1n02x5               g188(.a(new_n282), .b(new_n283), .c(new_n278), .o(new_n284));
  aobi12aa1n02x7               g189(.a(new_n281), .b(new_n284), .c(new_n274), .out0(\s[25] ));
  xorc02aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  nor042aa1n03x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  inv000aa1n02x5               g193(.a(new_n279), .o1(new_n289));
  inv000aa1n03x5               g194(.a(new_n287), .o1(new_n290));
  aoai13aa1n02x7               g195(.a(new_n290), .b(new_n283), .c(new_n274), .d(new_n289), .o1(new_n291));
  aoi022aa1n03x5               g196(.a(new_n291), .b(new_n286), .c(new_n281), .d(new_n288), .o1(\s[26] ));
  and002aa1n06x5               g197(.a(new_n286), .b(new_n280), .o(new_n293));
  inv000aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  nano32aa1n03x7               g199(.a(new_n294), .b(new_n235), .c(new_n272), .d(new_n254), .out0(new_n295));
  oai012aa1n06x5               g200(.a(new_n295), .b(new_n208), .c(new_n207), .o1(new_n296));
  oaoi03aa1n02x5               g201(.a(\a[26] ), .b(\b[25] ), .c(new_n290), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n297), .o1(new_n298));
  aoai13aa1n12x5               g203(.a(new_n298), .b(new_n294), .c(new_n282), .d(new_n278), .o1(new_n299));
  xorc02aa1n12x5               g204(.a(\a[27] ), .b(\b[26] ), .out0(new_n300));
  aoai13aa1n06x5               g205(.a(new_n300), .b(new_n299), .c(new_n198), .d(new_n295), .o1(new_n301));
  aoi112aa1n02x5               g206(.a(new_n300), .b(new_n297), .c(new_n279), .d(new_n293), .o1(new_n302));
  aobi12aa1n02x7               g207(.a(new_n301), .b(new_n302), .c(new_n296), .out0(\s[27] ));
  tech160nm_fixorc02aa1n03p5x5 g208(.a(\a[28] ), .b(\b[27] ), .out0(new_n304));
  nor042aa1d18x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n304), .b(new_n305), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(\b[25] ), .b(\a[26] ), .o1(new_n307));
  oai022aa1n02x5               g212(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n308));
  aoi022aa1n12x5               g213(.a(new_n279), .b(new_n293), .c(new_n307), .d(new_n308), .o1(new_n309));
  inv040aa1n08x5               g214(.a(new_n305), .o1(new_n310));
  inv000aa1n02x5               g215(.a(new_n300), .o1(new_n311));
  aoai13aa1n02x7               g216(.a(new_n310), .b(new_n311), .c(new_n296), .d(new_n309), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n312), .b(new_n304), .c(new_n301), .d(new_n306), .o1(\s[28] ));
  and002aa1n02x5               g218(.a(new_n304), .b(new_n300), .o(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n299), .c(new_n198), .d(new_n295), .o1(new_n315));
  tech160nm_fixorc02aa1n03p5x5 g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  oao003aa1n03x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n310), .carry(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n314), .o1(new_n319));
  aoai13aa1n02x7               g224(.a(new_n317), .b(new_n319), .c(new_n296), .d(new_n309), .o1(new_n320));
  aoi022aa1n03x5               g225(.a(new_n320), .b(new_n316), .c(new_n315), .d(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g227(.a(new_n311), .b(new_n304), .c(new_n316), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n299), .c(new_n198), .d(new_n295), .o1(new_n324));
  tech160nm_fixorc02aa1n05x5   g229(.a(\a[30] ), .b(\b[29] ), .out0(new_n325));
  and002aa1n02x5               g230(.a(\b[28] ), .b(\a[29] ), .o(new_n326));
  oabi12aa1n02x5               g231(.a(new_n325), .b(\a[29] ), .c(\b[28] ), .out0(new_n327));
  oab012aa1n02x4               g232(.a(new_n327), .b(new_n317), .c(new_n326), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n323), .o1(new_n329));
  oaoi03aa1n03x5               g234(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n330), .o1(new_n331));
  aoai13aa1n02x7               g236(.a(new_n331), .b(new_n329), .c(new_n296), .d(new_n309), .o1(new_n332));
  aoi022aa1n03x5               g237(.a(new_n332), .b(new_n325), .c(new_n324), .d(new_n328), .o1(\s[30] ));
  nano32aa1n02x5               g238(.a(new_n311), .b(new_n325), .c(new_n304), .d(new_n316), .out0(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n299), .c(new_n198), .d(new_n295), .o1(new_n335));
  xorc02aa1n02x5               g240(.a(\a[31] ), .b(\b[30] ), .out0(new_n336));
  inv000aa1d42x5               g241(.a(\a[30] ), .o1(new_n337));
  inv000aa1d42x5               g242(.a(\b[29] ), .o1(new_n338));
  oabi12aa1n02x5               g243(.a(new_n336), .b(\a[30] ), .c(\b[29] ), .out0(new_n339));
  oaoi13aa1n04x5               g244(.a(new_n339), .b(new_n330), .c(new_n337), .d(new_n338), .o1(new_n340));
  inv000aa1n02x5               g245(.a(new_n334), .o1(new_n341));
  oaoi03aa1n03x5               g246(.a(new_n337), .b(new_n338), .c(new_n330), .o1(new_n342));
  aoai13aa1n02x7               g247(.a(new_n342), .b(new_n341), .c(new_n296), .d(new_n309), .o1(new_n343));
  aoi022aa1n03x5               g248(.a(new_n343), .b(new_n336), .c(new_n335), .d(new_n340), .o1(\s[31] ));
  norb02aa1n02x5               g249(.a(new_n103), .b(new_n105), .out0(new_n345));
  xobna2aa1n03x5               g250(.a(new_n345), .b(new_n101), .c(new_n99), .out0(\s[3] ));
  aob012aa1n02x5               g251(.a(new_n345), .b(new_n101), .c(new_n99), .out0(new_n347));
  nanb02aa1n02x5               g252(.a(new_n108), .b(new_n104), .out0(new_n348));
  xnbna2aa1n03x5               g253(.a(new_n348), .b(new_n347), .c(new_n103), .out0(\s[4] ));
  oai022aa1n02x5               g254(.a(\a[4] ), .b(\b[3] ), .c(\b[4] ), .d(\a[5] ), .o1(new_n350));
  aoi122aa1n02x5               g255(.a(new_n350), .b(\a[5] ), .c(\b[4] ), .d(new_n105), .e(new_n104), .o1(new_n351));
  nanp02aa1n02x5               g256(.a(new_n107), .b(new_n351), .o1(new_n352));
  oai012aa1n02x5               g257(.a(new_n352), .b(new_n206), .c(new_n115), .o1(\s[5] ));
  xnbna2aa1n03x5               g258(.a(new_n112), .b(new_n352), .c(new_n114), .out0(\s[6] ));
  aoai13aa1n02x5               g259(.a(new_n121), .b(new_n112), .c(new_n352), .d(new_n114), .o1(new_n355));
  aoai13aa1n02x5               g260(.a(new_n110), .b(new_n111), .c(new_n352), .d(new_n114), .o1(new_n356));
  aobi12aa1n02x5               g261(.a(new_n355), .b(new_n356), .c(new_n119), .out0(\s[7] ));
  xnbna2aa1n03x5               g262(.a(new_n116), .b(new_n355), .c(new_n123), .out0(\s[8] ));
  tech160nm_fiao0012aa1n02p5x5 g263(.a(new_n120), .b(new_n107), .c(new_n109), .o(new_n359));
  aoi113aa1n02x5               g264(.a(new_n130), .b(new_n124), .c(new_n121), .d(new_n116), .e(new_n122), .o1(new_n360));
  aoi022aa1n02x5               g265(.a(new_n359), .b(new_n360), .c(new_n126), .d(new_n130), .o1(\s[9] ));
endmodule

