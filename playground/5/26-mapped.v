// Benchmark "adder" written by ABC on Wed Jul 17 14:42:20 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n343, new_n344,
    new_n345, new_n346, new_n348, new_n349, new_n350, new_n353, new_n354,
    new_n356, new_n357, new_n358, new_n360, new_n361, new_n362, new_n364,
    new_n365, new_n366;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  inv000aa1d42x5               g002(.a(\a[2] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[1] ), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  oao003aa1n03x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .carry(new_n101));
  nor022aa1n08x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor022aa1n16x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand22aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano23aa1n06x5               g010(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n106));
  nanp02aa1n03x5               g011(.a(new_n106), .b(new_n101), .o1(new_n107));
  tech160nm_fioai012aa1n02p5x5 g012(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n108));
  nand02aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor002aa1n03x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n06x5               g017(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n113));
  xnrc02aa1n12x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  inv000aa1d42x5               g019(.a(new_n114), .o1(new_n115));
  tech160nm_fixorc02aa1n05x5   g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  nand23aa1n03x5               g021(.a(new_n115), .b(new_n113), .c(new_n116), .o1(new_n117));
  nor042aa1n04x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  inv000aa1n02x5               g023(.a(new_n118), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  aoi012aa1n02x5               g025(.a(new_n110), .b(new_n111), .c(new_n109), .o1(new_n121));
  aobi12aa1n06x5               g026(.a(new_n121), .b(new_n113), .c(new_n120), .out0(new_n122));
  aoai13aa1n06x5               g027(.a(new_n122), .b(new_n117), .c(new_n107), .d(new_n108), .o1(new_n123));
  nor042aa1n04x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  nanb02aa1n02x5               g030(.a(new_n124), .b(new_n125), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(new_n123), .b(new_n127), .o1(new_n128));
  nor042aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1n03x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanb02aa1n02x5               g035(.a(new_n129), .b(new_n130), .out0(new_n131));
  xobna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n97), .out0(\s[10] ));
  nona22aa1n02x4               g037(.a(new_n123), .b(new_n126), .c(new_n131), .out0(new_n133));
  aoi012aa1n02x7               g038(.a(new_n129), .b(new_n124), .c(new_n130), .o1(new_n134));
  nor042aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x7               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n133), .c(new_n134), .out0(\s[11] ));
  aob012aa1n02x5               g043(.a(new_n137), .b(new_n133), .c(new_n134), .out0(new_n139));
  nor002aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n03x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n03x4               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  inv000aa1d42x5               g047(.a(\a[11] ), .o1(new_n143));
  inv000aa1d42x5               g048(.a(\b[10] ), .o1(new_n144));
  aboi22aa1n03x5               g049(.a(new_n140), .b(new_n141), .c(new_n143), .d(new_n144), .out0(new_n145));
  inv030aa1n03x5               g050(.a(new_n135), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n139), .b(new_n146), .o1(new_n147));
  aoi022aa1n02x5               g052(.a(new_n147), .b(new_n142), .c(new_n139), .d(new_n145), .o1(\s[12] ));
  nona23aa1n02x4               g053(.a(new_n130), .b(new_n125), .c(new_n124), .d(new_n129), .out0(new_n149));
  nano22aa1n03x7               g054(.a(new_n149), .b(new_n137), .c(new_n142), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n123), .b(new_n150), .o1(new_n151));
  inv000aa1n03x5               g056(.a(new_n134), .o1(new_n152));
  nano23aa1n03x5               g057(.a(new_n135), .b(new_n140), .c(new_n141), .d(new_n136), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n153), .b(new_n152), .o1(new_n154));
  oaoi03aa1n12x5               g059(.a(\a[12] ), .b(\b[11] ), .c(new_n146), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(new_n154), .b(new_n156), .o1(new_n157));
  nanb02aa1n03x5               g062(.a(new_n157), .b(new_n151), .out0(new_n158));
  xnrc02aa1n03x5               g063(.a(\b[12] ), .b(\a[13] ), .out0(new_n159));
  and003aa1n02x5               g064(.a(new_n154), .b(new_n159), .c(new_n156), .o(new_n160));
  aboi22aa1n03x5               g065(.a(new_n159), .b(new_n158), .c(new_n160), .d(new_n151), .out0(\s[13] ));
  nanb02aa1n02x5               g066(.a(new_n159), .b(new_n158), .out0(new_n162));
  xnrc02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .out0(new_n163));
  norp02aa1n02x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  oai012aa1n02x5               g070(.a(new_n162), .b(\b[12] ), .c(\a[13] ), .o1(new_n166));
  aboi22aa1n03x5               g071(.a(new_n163), .b(new_n166), .c(new_n162), .d(new_n165), .out0(\s[14] ));
  nor002aa1n02x5               g072(.a(new_n163), .b(new_n159), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n157), .c(new_n123), .d(new_n150), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  aoi012aa1n02x5               g076(.a(new_n170), .b(new_n164), .c(new_n171), .o1(new_n172));
  nor002aa1d24x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nanp02aa1n04x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n06x4               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n169), .c(new_n172), .out0(\s[15] ));
  aob012aa1n03x5               g081(.a(new_n175), .b(new_n169), .c(new_n172), .out0(new_n177));
  nor022aa1n08x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand02aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aoib12aa1n02x5               g085(.a(new_n173), .b(new_n179), .c(new_n178), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n173), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n175), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n182), .b(new_n183), .c(new_n169), .d(new_n172), .o1(new_n184));
  aoi022aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n177), .d(new_n181), .o1(\s[16] ));
  nona23aa1d24x5               g090(.a(new_n179), .b(new_n174), .c(new_n173), .d(new_n178), .out0(new_n186));
  nona32aa1n09x5               g091(.a(new_n150), .b(new_n186), .c(new_n163), .d(new_n159), .out0(new_n187));
  nanb02aa1n06x5               g092(.a(new_n187), .b(new_n123), .out0(new_n188));
  inv000aa1d42x5               g093(.a(new_n186), .o1(new_n189));
  aoai13aa1n06x5               g094(.a(new_n168), .b(new_n155), .c(new_n153), .d(new_n152), .o1(new_n190));
  aob012aa1n06x5               g095(.a(new_n189), .b(new_n190), .c(new_n172), .out0(new_n191));
  oai012aa1n02x5               g096(.a(new_n179), .b(new_n178), .c(new_n173), .o1(new_n192));
  nand23aa1n06x5               g097(.a(new_n188), .b(new_n191), .c(new_n192), .o1(new_n193));
  tech160nm_fixorc02aa1n03p5x5 g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  nano22aa1n02x4               g099(.a(new_n194), .b(new_n191), .c(new_n192), .out0(new_n195));
  aoi022aa1n02x5               g100(.a(new_n195), .b(new_n188), .c(new_n193), .d(new_n194), .o1(\s[17] ));
  inv040aa1d32x5               g101(.a(\a[17] ), .o1(new_n197));
  inv040aa1d28x5               g102(.a(\b[16] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(new_n198), .b(new_n197), .o1(new_n199));
  aobi12aa1n02x5               g104(.a(new_n108), .b(new_n106), .c(new_n101), .out0(new_n200));
  oaoi13aa1n09x5               g105(.a(new_n187), .b(new_n122), .c(new_n200), .d(new_n117), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n192), .b(new_n186), .c(new_n190), .d(new_n172), .o1(new_n202));
  oaih12aa1n02x5               g107(.a(new_n194), .b(new_n202), .c(new_n201), .o1(new_n203));
  nor042aa1n02x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nanp02aa1n04x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  norb02aa1n06x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n203), .c(new_n199), .out0(\s[18] ));
  and002aa1n02x5               g112(.a(new_n194), .b(new_n206), .o(new_n208));
  oaih12aa1n02x5               g113(.a(new_n208), .b(new_n202), .c(new_n201), .o1(new_n209));
  aoi013aa1n06x4               g114(.a(new_n204), .b(new_n205), .c(new_n197), .d(new_n198), .o1(new_n210));
  nor002aa1d32x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nand02aa1n06x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n209), .c(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n12x5               g120(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n216));
  aoai13aa1n03x5               g121(.a(new_n213), .b(new_n216), .c(new_n193), .d(new_n208), .o1(new_n217));
  nor022aa1n16x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1n06x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  inv000aa1d42x5               g125(.a(\a[19] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[18] ), .o1(new_n222));
  aboi22aa1n03x5               g127(.a(new_n218), .b(new_n219), .c(new_n221), .d(new_n222), .out0(new_n223));
  inv040aa1n08x5               g128(.a(new_n211), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n213), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n224), .b(new_n225), .c(new_n209), .d(new_n210), .o1(new_n226));
  aoi022aa1n03x5               g131(.a(new_n226), .b(new_n220), .c(new_n217), .d(new_n223), .o1(\s[20] ));
  nona23aa1d18x5               g132(.a(new_n219), .b(new_n212), .c(new_n211), .d(new_n218), .out0(new_n228));
  nano22aa1n12x5               g133(.a(new_n228), .b(new_n194), .c(new_n206), .out0(new_n229));
  oaih12aa1n02x5               g134(.a(new_n229), .b(new_n202), .c(new_n201), .o1(new_n230));
  oaoi03aa1n12x5               g135(.a(\a[20] ), .b(\b[19] ), .c(new_n224), .o1(new_n231));
  inv030aa1n02x5               g136(.a(new_n231), .o1(new_n232));
  oai012aa1n18x5               g137(.a(new_n232), .b(new_n228), .c(new_n210), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(new_n230), .b(new_n234), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nano23aa1n03x7               g142(.a(new_n211), .b(new_n218), .c(new_n219), .d(new_n212), .out0(new_n238));
  aoi112aa1n02x5               g143(.a(new_n231), .b(new_n237), .c(new_n238), .d(new_n216), .o1(new_n239));
  aoi022aa1n02x5               g144(.a(new_n235), .b(new_n237), .c(new_n230), .d(new_n239), .o1(\s[21] ));
  aoai13aa1n04x5               g145(.a(new_n237), .b(new_n233), .c(new_n193), .d(new_n229), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[21] ), .b(\a[22] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  nor042aa1n06x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n242), .b(new_n244), .out0(new_n245));
  inv000aa1n09x5               g150(.a(new_n244), .o1(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n236), .c(new_n230), .d(new_n234), .o1(new_n247));
  aoi022aa1n03x5               g152(.a(new_n247), .b(new_n243), .c(new_n241), .d(new_n245), .o1(\s[22] ));
  nor042aa1n06x5               g153(.a(new_n242), .b(new_n236), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n228), .b(new_n249), .c(new_n194), .d(new_n206), .out0(new_n250));
  oaih12aa1n02x5               g155(.a(new_n250), .b(new_n202), .c(new_n201), .o1(new_n251));
  oaoi03aa1n12x5               g156(.a(\a[22] ), .b(\b[21] ), .c(new_n246), .o1(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(new_n233), .c(new_n249), .o1(new_n253));
  nanp02aa1n02x5               g158(.a(new_n251), .b(new_n253), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  aoi112aa1n02x5               g160(.a(new_n255), .b(new_n252), .c(new_n233), .d(new_n249), .o1(new_n256));
  aoi022aa1n02x5               g161(.a(new_n254), .b(new_n255), .c(new_n251), .d(new_n256), .o1(\s[23] ));
  inv000aa1n02x5               g162(.a(new_n253), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n255), .b(new_n258), .c(new_n193), .d(new_n250), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  nor042aa1n06x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  norp02aa1n02x5               g166(.a(new_n260), .b(new_n261), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n261), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n255), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n263), .b(new_n264), .c(new_n251), .d(new_n253), .o1(new_n265));
  aoi022aa1n03x5               g170(.a(new_n265), .b(new_n260), .c(new_n259), .d(new_n262), .o1(\s[24] ));
  inv000aa1d42x5               g171(.a(new_n229), .o1(new_n267));
  and002aa1n12x5               g172(.a(new_n260), .b(new_n255), .o(new_n268));
  nano22aa1n02x4               g173(.a(new_n267), .b(new_n268), .c(new_n249), .out0(new_n269));
  oaih12aa1n02x5               g174(.a(new_n269), .b(new_n202), .c(new_n201), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n249), .b(new_n231), .c(new_n238), .d(new_n216), .o1(new_n271));
  inv000aa1n02x5               g176(.a(new_n252), .o1(new_n272));
  inv000aa1n03x5               g177(.a(new_n268), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .c(new_n263), .carry(new_n274));
  aoai13aa1n12x5               g179(.a(new_n274), .b(new_n273), .c(new_n271), .d(new_n272), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(new_n270), .b(new_n276), .o1(new_n277));
  xorc02aa1n12x5               g182(.a(\a[25] ), .b(\b[24] ), .out0(new_n278));
  aoai13aa1n03x5               g183(.a(new_n268), .b(new_n252), .c(new_n233), .d(new_n249), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n278), .o1(new_n280));
  and003aa1n02x5               g185(.a(new_n279), .b(new_n280), .c(new_n274), .o(new_n281));
  aoi022aa1n02x5               g186(.a(new_n277), .b(new_n278), .c(new_n270), .d(new_n281), .o1(\s[25] ));
  aoai13aa1n03x5               g187(.a(new_n278), .b(new_n275), .c(new_n193), .d(new_n269), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .out0(new_n284));
  nor042aa1n03x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n284), .b(new_n285), .o1(new_n286));
  inv000aa1n03x5               g191(.a(new_n285), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n280), .c(new_n270), .d(new_n276), .o1(new_n288));
  aoi022aa1n03x5               g193(.a(new_n288), .b(new_n284), .c(new_n283), .d(new_n286), .o1(\s[26] ));
  and002aa1n02x5               g194(.a(new_n284), .b(new_n278), .o(new_n290));
  inv000aa1n02x5               g195(.a(new_n290), .o1(new_n291));
  nano32aa1n03x7               g196(.a(new_n291), .b(new_n229), .c(new_n268), .d(new_n249), .out0(new_n292));
  tech160nm_fioai012aa1n04x5   g197(.a(new_n292), .b(new_n202), .c(new_n201), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(\b[25] ), .b(\a[26] ), .o1(new_n294));
  oai022aa1n02x5               g199(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n295));
  aoi022aa1n12x5               g200(.a(new_n275), .b(new_n290), .c(new_n294), .d(new_n295), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n293), .b(new_n296), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  oaoi03aa1n02x5               g203(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .o1(new_n299));
  aoi112aa1n02x5               g204(.a(new_n298), .b(new_n299), .c(new_n275), .d(new_n290), .o1(new_n300));
  aoi022aa1n02x5               g205(.a(new_n297), .b(new_n298), .c(new_n293), .d(new_n300), .o1(\s[27] ));
  inv000aa1d42x5               g206(.a(new_n299), .o1(new_n302));
  aoai13aa1n02x7               g207(.a(new_n302), .b(new_n291), .c(new_n279), .d(new_n274), .o1(new_n303));
  aoai13aa1n02x7               g208(.a(new_n298), .b(new_n303), .c(new_n193), .d(new_n292), .o1(new_n304));
  xorc02aa1n12x5               g209(.a(\a[28] ), .b(\b[27] ), .out0(new_n305));
  nor042aa1d18x5               g210(.a(\b[26] ), .b(\a[27] ), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n305), .b(new_n306), .o1(new_n307));
  inv040aa1n08x5               g212(.a(new_n306), .o1(new_n308));
  inv020aa1n03x5               g213(.a(new_n298), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n308), .b(new_n309), .c(new_n293), .d(new_n296), .o1(new_n310));
  aoi022aa1n03x5               g215(.a(new_n310), .b(new_n305), .c(new_n304), .d(new_n307), .o1(\s[28] ));
  and002aa1n02x5               g216(.a(new_n305), .b(new_n298), .o(new_n312));
  aoai13aa1n02x5               g217(.a(new_n312), .b(new_n303), .c(new_n193), .d(new_n292), .o1(new_n313));
  tech160nm_fixorc02aa1n03p5x5 g218(.a(\a[29] ), .b(\b[28] ), .out0(new_n314));
  oao003aa1n03x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n308), .carry(new_n315));
  norb02aa1n02x5               g220(.a(new_n315), .b(new_n314), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n312), .o1(new_n317));
  aoai13aa1n03x5               g222(.a(new_n315), .b(new_n317), .c(new_n293), .d(new_n296), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n318), .b(new_n314), .c(new_n313), .d(new_n316), .o1(\s[29] ));
  xorb03aa1n02x5               g224(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g225(.a(new_n309), .b(new_n305), .c(new_n314), .out0(new_n321));
  aoai13aa1n02x5               g226(.a(new_n321), .b(new_n303), .c(new_n193), .d(new_n292), .o1(new_n322));
  tech160nm_fixorc02aa1n03p5x5 g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  and002aa1n02x5               g228(.a(\b[28] ), .b(\a[29] ), .o(new_n324));
  oabi12aa1n02x5               g229(.a(new_n323), .b(\a[29] ), .c(\b[28] ), .out0(new_n325));
  oab012aa1n02x4               g230(.a(new_n325), .b(new_n315), .c(new_n324), .out0(new_n326));
  inv000aa1d42x5               g231(.a(new_n321), .o1(new_n327));
  tech160nm_fioaoi03aa1n02p5x5 g232(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .o1(new_n328));
  inv000aa1d42x5               g233(.a(new_n328), .o1(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n327), .c(new_n293), .d(new_n296), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n330), .b(new_n323), .c(new_n322), .d(new_n326), .o1(\s[30] ));
  nano32aa1n02x4               g236(.a(new_n309), .b(new_n323), .c(new_n305), .d(new_n314), .out0(new_n332));
  aoai13aa1n02x5               g237(.a(new_n332), .b(new_n303), .c(new_n193), .d(new_n292), .o1(new_n333));
  xorc02aa1n02x5               g238(.a(\a[31] ), .b(\b[30] ), .out0(new_n334));
  inv000aa1d42x5               g239(.a(\a[30] ), .o1(new_n335));
  inv000aa1d42x5               g240(.a(\b[29] ), .o1(new_n336));
  oabi12aa1n02x5               g241(.a(new_n334), .b(\a[30] ), .c(\b[29] ), .out0(new_n337));
  oaoi13aa1n03x5               g242(.a(new_n337), .b(new_n328), .c(new_n335), .d(new_n336), .o1(new_n338));
  inv000aa1n02x5               g243(.a(new_n332), .o1(new_n339));
  oaoi03aa1n02x5               g244(.a(new_n335), .b(new_n336), .c(new_n328), .o1(new_n340));
  aoai13aa1n03x5               g245(.a(new_n340), .b(new_n339), .c(new_n293), .d(new_n296), .o1(new_n341));
  aoi022aa1n03x5               g246(.a(new_n341), .b(new_n334), .c(new_n333), .d(new_n338), .o1(\s[31] ));
  aoi022aa1n02x5               g247(.a(new_n99), .b(new_n98), .c(\a[1] ), .d(\b[0] ), .o1(new_n343));
  oaib12aa1n02x5               g248(.a(new_n343), .b(new_n99), .c(\a[2] ), .out0(new_n344));
  norb02aa1n02x5               g249(.a(new_n105), .b(new_n104), .out0(new_n345));
  aboi22aa1n03x5               g250(.a(new_n104), .b(new_n105), .c(new_n98), .d(new_n99), .out0(new_n346));
  aoi022aa1n02x5               g251(.a(new_n101), .b(new_n345), .c(new_n344), .d(new_n346), .o1(\s[3] ));
  norb02aa1n02x5               g252(.a(new_n103), .b(new_n102), .out0(new_n348));
  aoai13aa1n02x5               g253(.a(new_n348), .b(new_n104), .c(new_n101), .d(new_n105), .o1(new_n349));
  aoi112aa1n02x5               g254(.a(new_n104), .b(new_n348), .c(new_n101), .d(new_n105), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n349), .b(new_n350), .out0(\s[4] ));
  xnbna2aa1n03x5               g256(.a(new_n116), .b(new_n107), .c(new_n108), .out0(\s[5] ));
  nanp02aa1n02x5               g257(.a(new_n107), .b(new_n108), .o1(new_n353));
  nanp02aa1n02x5               g258(.a(new_n353), .b(new_n116), .o1(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n115), .b(new_n354), .c(new_n119), .out0(\s[6] ));
  norb02aa1n02x5               g260(.a(new_n112), .b(new_n111), .out0(new_n356));
  nanp02aa1n02x5               g261(.a(\b[5] ), .b(\a[6] ), .o1(new_n357));
  nona22aa1n02x4               g262(.a(new_n354), .b(new_n118), .c(new_n114), .out0(new_n358));
  xobna2aa1n03x5               g263(.a(new_n356), .b(new_n358), .c(new_n357), .out0(\s[7] ));
  norb02aa1n02x5               g264(.a(new_n109), .b(new_n110), .out0(new_n360));
  aoi013aa1n02x4               g265(.a(new_n111), .b(new_n358), .c(new_n357), .d(new_n112), .o1(new_n361));
  aoi113aa1n02x5               g266(.a(new_n111), .b(new_n360), .c(new_n358), .d(new_n357), .e(new_n356), .o1(new_n362));
  aoib12aa1n02x5               g267(.a(new_n362), .b(new_n360), .c(new_n361), .out0(\s[8] ));
  aoi012aa1n02x5               g268(.a(new_n117), .b(new_n107), .c(new_n108), .o1(new_n364));
  oaib12aa1n02x5               g269(.a(new_n121), .b(new_n124), .c(new_n125), .out0(new_n365));
  aoi012aa1n02x5               g270(.a(new_n365), .b(new_n113), .c(new_n120), .o1(new_n366));
  aboi22aa1n03x5               g271(.a(new_n364), .b(new_n366), .c(new_n123), .d(new_n127), .out0(\s[9] ));
endmodule


