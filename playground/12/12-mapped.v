// Benchmark "adder" written by ABC on Wed Jul 17 18:10:44 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n316,
    new_n318, new_n319, new_n320, new_n322, new_n323, new_n324, new_n326,
    new_n327, new_n329, new_n330, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1d18x5               g002(.a(\b[4] ), .b(\a[5] ), .o1(new_n98));
  inv040aa1n08x5               g003(.a(new_n98), .o1(new_n99));
  oaoi03aa1n12x5               g004(.a(\a[6] ), .b(\b[5] ), .c(new_n99), .o1(new_n100));
  xnrc02aa1n12x5               g005(.a(\b[7] ), .b(\a[8] ), .out0(new_n101));
  xnrc02aa1n12x5               g006(.a(\b[6] ), .b(\a[7] ), .out0(new_n102));
  nor022aa1n08x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(new_n104), .o1(new_n105));
  tech160nm_fioaoi03aa1n03p5x5 g010(.a(\a[8] ), .b(\b[7] ), .c(new_n105), .o1(new_n106));
  aoi012aa1d18x5               g011(.a(new_n106), .b(new_n103), .c(new_n100), .o1(new_n107));
  oa0022aa1n06x5               g012(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n108));
  xnrc02aa1n12x5               g013(.a(\b[2] ), .b(\a[3] ), .out0(new_n109));
  nand42aa1n08x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  nor042aa1n12x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nand02aa1d28x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  oai012aa1d24x5               g017(.a(new_n110), .b(new_n111), .c(new_n112), .o1(new_n113));
  oaih12aa1n06x5               g018(.a(new_n108), .b(new_n109), .c(new_n113), .o1(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  nand42aa1d28x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  tech160nm_fioai012aa1n04x5   g021(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .o1(new_n117));
  aoi112aa1n03x5               g022(.a(new_n115), .b(new_n117), .c(\a[5] ), .d(\b[4] ), .o1(new_n118));
  nand23aa1n06x5               g023(.a(new_n114), .b(new_n118), .c(new_n103), .o1(new_n119));
  nand02aa1d16x5               g024(.a(new_n119), .b(new_n107), .o1(new_n120));
  xorc02aa1n12x5               g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  nor042aa1n06x5               g026(.a(\b[9] ), .b(\a[10] ), .o1(new_n122));
  nand02aa1d24x5               g027(.a(\b[9] ), .b(\a[10] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n123), .b(new_n122), .out0(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n97), .c(new_n120), .d(new_n121), .o1(new_n125));
  aoi112aa1n02x5               g030(.a(new_n124), .b(new_n97), .c(new_n120), .d(new_n121), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n125), .b(new_n126), .out0(\s[10] ));
  tech160nm_fiaoi012aa1n04x5   g032(.a(new_n122), .b(new_n97), .c(new_n123), .o1(new_n128));
  xnrc02aa1n12x5               g033(.a(\b[10] ), .b(\a[11] ), .out0(new_n129));
  inv030aa1n06x5               g034(.a(new_n129), .o1(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n125), .c(new_n128), .out0(\s[11] ));
  aob012aa1n03x5               g036(.a(new_n130), .b(new_n125), .c(new_n128), .out0(new_n132));
  xnrc02aa1n12x5               g037(.a(\b[11] ), .b(\a[12] ), .out0(new_n133));
  nor042aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  inv000aa1n02x5               g040(.a(new_n134), .o1(new_n136));
  aoai13aa1n02x7               g041(.a(new_n136), .b(new_n129), .c(new_n125), .d(new_n128), .o1(new_n137));
  aboi22aa1n03x5               g042(.a(new_n133), .b(new_n137), .c(new_n132), .d(new_n135), .out0(\s[12] ));
  nanb02aa1n02x5               g043(.a(new_n122), .b(new_n123), .out0(new_n139));
  nona23aa1n09x5               g044(.a(new_n130), .b(new_n121), .c(new_n133), .d(new_n139), .out0(new_n140));
  nanb02aa1n02x5               g045(.a(new_n140), .b(new_n120), .out0(new_n141));
  nona22aa1n06x5               g046(.a(new_n130), .b(new_n133), .c(new_n128), .out0(new_n142));
  oao003aa1n03x5               g047(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .carry(new_n143));
  and002aa1n06x5               g048(.a(new_n142), .b(new_n143), .o(new_n144));
  xorc02aa1n02x5               g049(.a(\a[13] ), .b(\b[12] ), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n141), .c(new_n144), .out0(\s[13] ));
  aoai13aa1n04x5               g051(.a(new_n144), .b(new_n140), .c(new_n119), .d(new_n107), .o1(new_n147));
  nor042aa1n09x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  xorc02aa1n02x5               g053(.a(\a[14] ), .b(\b[13] ), .out0(new_n149));
  aoi112aa1n02x5               g054(.a(new_n148), .b(new_n149), .c(new_n147), .d(new_n145), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n149), .b(new_n148), .c(new_n147), .d(new_n145), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(\s[14] ));
  inv030aa1d32x5               g057(.a(\a[13] ), .o1(new_n153));
  inv040aa1d32x5               g058(.a(\a[14] ), .o1(new_n154));
  xroi22aa1d06x4               g059(.a(new_n153), .b(\b[12] ), .c(new_n154), .d(\b[13] ), .out0(new_n155));
  aob012aa1n03x5               g060(.a(new_n148), .b(\b[13] ), .c(\a[14] ), .out0(new_n156));
  oaib12aa1n09x5               g061(.a(new_n156), .b(\b[13] ), .c(new_n154), .out0(new_n157));
  xorc02aa1n12x5               g062(.a(\a[15] ), .b(\b[14] ), .out0(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n157), .c(new_n147), .d(new_n155), .o1(new_n159));
  aoi112aa1n02x5               g064(.a(new_n158), .b(new_n157), .c(new_n147), .d(new_n155), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(\s[15] ));
  xorc02aa1n12x5               g066(.a(\a[16] ), .b(\b[15] ), .out0(new_n162));
  nor002aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  norp02aa1n02x5               g068(.a(new_n162), .b(new_n163), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\a[15] ), .o1(new_n165));
  oaib12aa1n06x5               g070(.a(new_n159), .b(\b[14] ), .c(new_n165), .out0(new_n166));
  aoi022aa1n02x7               g071(.a(new_n166), .b(new_n162), .c(new_n159), .d(new_n164), .o1(\s[16] ));
  inv040aa1d28x5               g072(.a(\a[16] ), .o1(new_n168));
  xroi22aa1d06x4               g073(.a(new_n165), .b(\b[14] ), .c(new_n168), .d(\b[15] ), .out0(new_n169));
  nano22aa1n12x5               g074(.a(new_n140), .b(new_n155), .c(new_n169), .out0(new_n170));
  nand02aa1d06x5               g075(.a(new_n120), .b(new_n170), .o1(new_n171));
  nand22aa1n03x5               g076(.a(new_n169), .b(new_n155), .o1(new_n172));
  aob012aa1n03x5               g077(.a(new_n163), .b(\b[15] ), .c(\a[16] ), .out0(new_n173));
  oaib12aa1n02x5               g078(.a(new_n173), .b(\b[15] ), .c(new_n168), .out0(new_n174));
  aoi013aa1n06x4               g079(.a(new_n174), .b(new_n157), .c(new_n158), .d(new_n162), .o1(new_n175));
  aoai13aa1n12x5               g080(.a(new_n175), .b(new_n172), .c(new_n142), .d(new_n143), .o1(new_n176));
  nanb02aa1n09x5               g081(.a(new_n176), .b(new_n171), .out0(new_n177));
  xorc02aa1n12x5               g082(.a(\a[17] ), .b(\b[16] ), .out0(new_n178));
  aoi012aa1n02x5               g083(.a(new_n172), .b(new_n142), .c(new_n143), .o1(new_n179));
  norb03aa1n02x5               g084(.a(new_n175), .b(new_n179), .c(new_n178), .out0(new_n180));
  aoi022aa1n02x5               g085(.a(new_n177), .b(new_n178), .c(new_n171), .d(new_n180), .o1(\s[17] ));
  inv040aa1d32x5               g086(.a(\a[17] ), .o1(new_n182));
  inv030aa1d32x5               g087(.a(\b[16] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n182), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n178), .b(new_n176), .c(new_n120), .d(new_n170), .o1(new_n185));
  nor022aa1n08x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nand02aa1d28x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  norb02aa1n06x4               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n185), .c(new_n184), .out0(\s[18] ));
  and002aa1n02x5               g094(.a(new_n178), .b(new_n188), .o(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n176), .c(new_n120), .d(new_n170), .o1(new_n191));
  aoi013aa1n09x5               g096(.a(new_n186), .b(new_n187), .c(new_n182), .d(new_n183), .o1(new_n192));
  nor002aa1d32x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n24x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n06x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n191), .c(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n03x5               g102(.a(new_n195), .b(new_n191), .c(new_n192), .out0(new_n198));
  nor002aa1d32x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand42aa1n16x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  inv000aa1d42x5               g106(.a(\a[19] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(\b[18] ), .o1(new_n203));
  aboi22aa1n03x5               g108(.a(new_n199), .b(new_n200), .c(new_n202), .d(new_n203), .out0(new_n204));
  inv040aa1n08x5               g109(.a(new_n193), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n195), .o1(new_n206));
  aoai13aa1n02x7               g111(.a(new_n205), .b(new_n206), .c(new_n191), .d(new_n192), .o1(new_n207));
  aoi022aa1n03x5               g112(.a(new_n207), .b(new_n201), .c(new_n198), .d(new_n204), .o1(\s[20] ));
  nano23aa1n06x5               g113(.a(new_n193), .b(new_n199), .c(new_n200), .d(new_n194), .out0(new_n209));
  nand23aa1d12x5               g114(.a(new_n209), .b(new_n178), .c(new_n188), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n176), .c(new_n120), .d(new_n170), .o1(new_n212));
  nona23aa1n09x5               g117(.a(new_n200), .b(new_n194), .c(new_n193), .d(new_n199), .out0(new_n213));
  oaoi03aa1n12x5               g118(.a(\a[20] ), .b(\b[19] ), .c(new_n205), .o1(new_n214));
  inv040aa1n04x5               g119(.a(new_n214), .o1(new_n215));
  oai012aa1n18x5               g120(.a(new_n215), .b(new_n213), .c(new_n192), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  xnrc02aa1n12x5               g122(.a(\b[20] ), .b(\a[21] ), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n212), .c(new_n217), .out0(\s[21] ));
  aob012aa1n03x5               g125(.a(new_n219), .b(new_n212), .c(new_n217), .out0(new_n221));
  tech160nm_fixnrc02aa1n04x5   g126(.a(\b[21] ), .b(\a[22] ), .out0(new_n222));
  nor042aa1n06x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n222), .b(new_n223), .out0(new_n224));
  inv040aa1n03x5               g129(.a(new_n223), .o1(new_n225));
  aoai13aa1n02x7               g130(.a(new_n225), .b(new_n218), .c(new_n212), .d(new_n217), .o1(new_n226));
  aboi22aa1n03x5               g131(.a(new_n222), .b(new_n226), .c(new_n221), .d(new_n224), .out0(\s[22] ));
  nor042aa1n06x5               g132(.a(new_n222), .b(new_n218), .o1(new_n228));
  norb02aa1n06x4               g133(.a(new_n228), .b(new_n210), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n176), .c(new_n120), .d(new_n170), .o1(new_n230));
  oaoi03aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .o1(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n216), .c(new_n228), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xnbna2aa1n03x5               g138(.a(new_n233), .b(new_n230), .c(new_n232), .out0(\s[23] ));
  aob012aa1n03x5               g139(.a(new_n233), .b(new_n230), .c(new_n232), .out0(new_n235));
  tech160nm_fixorc02aa1n04x5   g140(.a(\a[24] ), .b(\b[23] ), .out0(new_n236));
  nor042aa1d18x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  norp02aa1n02x5               g142(.a(new_n236), .b(new_n237), .o1(new_n238));
  inv000aa1n09x5               g143(.a(new_n237), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n233), .o1(new_n240));
  aoai13aa1n02x7               g145(.a(new_n239), .b(new_n240), .c(new_n230), .d(new_n232), .o1(new_n241));
  aoi022aa1n03x5               g146(.a(new_n241), .b(new_n236), .c(new_n235), .d(new_n238), .o1(\s[24] ));
  nano32aa1n03x7               g147(.a(new_n210), .b(new_n236), .c(new_n228), .d(new_n233), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n176), .c(new_n120), .d(new_n170), .o1(new_n244));
  oaoi03aa1n03x5               g149(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n228), .b(new_n214), .c(new_n209), .d(new_n245), .o1(new_n246));
  inv000aa1n02x5               g151(.a(new_n231), .o1(new_n247));
  and002aa1n12x5               g152(.a(new_n236), .b(new_n233), .o(new_n248));
  inv020aa1n04x5               g153(.a(new_n248), .o1(new_n249));
  oaoi03aa1n03x5               g154(.a(\a[24] ), .b(\b[23] ), .c(new_n239), .o1(new_n250));
  inv000aa1n03x5               g155(.a(new_n250), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n249), .c(new_n246), .d(new_n247), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xnbna2aa1n03x5               g159(.a(new_n254), .b(new_n244), .c(new_n253), .out0(\s[25] ));
  aob012aa1n03x5               g160(.a(new_n254), .b(new_n244), .c(new_n253), .out0(new_n256));
  tech160nm_fixorc02aa1n02p5x5 g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  norp02aa1n02x5               g163(.a(new_n257), .b(new_n258), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n258), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n254), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n260), .b(new_n261), .c(new_n244), .d(new_n253), .o1(new_n262));
  aoi022aa1n03x5               g167(.a(new_n262), .b(new_n257), .c(new_n256), .d(new_n259), .o1(\s[26] ));
  and002aa1n06x5               g168(.a(new_n257), .b(new_n254), .o(new_n264));
  inv000aa1n06x5               g169(.a(new_n264), .o1(new_n265));
  nano23aa1d15x5               g170(.a(new_n265), .b(new_n210), .c(new_n248), .d(new_n228), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n176), .c(new_n120), .d(new_n170), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n248), .b(new_n231), .c(new_n216), .d(new_n228), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(\b[25] ), .b(\a[26] ), .o1(new_n269));
  oai022aa1n02x5               g174(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(new_n270), .b(new_n269), .o1(new_n271));
  aoai13aa1n04x5               g176(.a(new_n271), .b(new_n265), .c(new_n268), .d(new_n251), .o1(new_n272));
  xorc02aa1n12x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n272), .c(new_n177), .d(new_n266), .o1(new_n274));
  aoi122aa1n02x7               g179(.a(new_n273), .b(new_n269), .c(new_n270), .d(new_n252), .e(new_n264), .o1(new_n275));
  aobi12aa1n02x7               g180(.a(new_n274), .b(new_n275), .c(new_n267), .out0(\s[27] ));
  tech160nm_fixorc02aa1n02p5x5 g181(.a(\a[28] ), .b(\b[27] ), .out0(new_n277));
  nor042aa1n03x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  norp02aa1n02x5               g183(.a(new_n277), .b(new_n278), .o1(new_n279));
  aoi022aa1n09x5               g184(.a(new_n252), .b(new_n264), .c(new_n269), .d(new_n270), .o1(new_n280));
  inv000aa1n03x5               g185(.a(new_n278), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n273), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n281), .b(new_n282), .c(new_n280), .d(new_n267), .o1(new_n283));
  aoi022aa1n03x5               g188(.a(new_n283), .b(new_n277), .c(new_n274), .d(new_n279), .o1(\s[28] ));
  and002aa1n02x5               g189(.a(new_n277), .b(new_n273), .o(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n272), .c(new_n177), .d(new_n266), .o1(new_n286));
  xorc02aa1n03x5               g191(.a(\a[29] ), .b(\b[28] ), .out0(new_n287));
  norp02aa1n02x5               g192(.a(\b[27] ), .b(\a[28] ), .o1(new_n288));
  aoi112aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n289));
  norp03aa1n02x5               g194(.a(new_n287), .b(new_n289), .c(new_n288), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n285), .o1(new_n291));
  oaoi03aa1n09x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  aoai13aa1n02x7               g198(.a(new_n293), .b(new_n291), .c(new_n280), .d(new_n267), .o1(new_n294));
  aoi022aa1n03x5               g199(.a(new_n294), .b(new_n287), .c(new_n286), .d(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d15x5               g201(.a(new_n282), .b(new_n277), .c(new_n287), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n272), .c(new_n177), .d(new_n266), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .out0(new_n299));
  inv000aa1d42x5               g204(.a(\a[29] ), .o1(new_n300));
  inv000aa1d42x5               g205(.a(\b[28] ), .o1(new_n301));
  oa0022aa1n02x5               g206(.a(new_n289), .b(new_n288), .c(new_n301), .d(new_n300), .o(new_n302));
  aoi112aa1n02x5               g207(.a(new_n302), .b(new_n299), .c(new_n300), .d(new_n301), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n297), .o1(new_n304));
  oaoi03aa1n02x5               g209(.a(new_n300), .b(new_n301), .c(new_n292), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n304), .c(new_n280), .d(new_n267), .o1(new_n306));
  aoi022aa1n03x5               g211(.a(new_n306), .b(new_n299), .c(new_n298), .d(new_n303), .o1(\s[30] ));
  nano32aa1n06x5               g212(.a(new_n282), .b(new_n299), .c(new_n277), .d(new_n287), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n272), .c(new_n177), .d(new_n266), .o1(new_n309));
  xorc02aa1n02x5               g214(.a(\a[31] ), .b(\b[30] ), .out0(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n311));
  norb02aa1n02x5               g216(.a(new_n311), .b(new_n310), .out0(new_n312));
  inv000aa1n02x5               g217(.a(new_n308), .o1(new_n313));
  aoai13aa1n03x5               g218(.a(new_n311), .b(new_n313), .c(new_n280), .d(new_n267), .o1(new_n314));
  aoi022aa1n03x5               g219(.a(new_n314), .b(new_n310), .c(new_n309), .d(new_n312), .o1(\s[31] ));
  inv000aa1d42x5               g220(.a(\a[3] ), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n113), .b(\b[2] ), .c(new_n316), .out0(\s[3] ));
  orn002aa1n02x5               g222(.a(new_n109), .b(new_n113), .o(new_n318));
  xorc02aa1n02x5               g223(.a(\a[4] ), .b(\b[3] ), .out0(new_n319));
  aoib12aa1n02x5               g224(.a(new_n319), .b(new_n316), .c(\b[2] ), .out0(new_n320));
  aoi022aa1n02x5               g225(.a(new_n318), .b(new_n320), .c(new_n114), .d(new_n319), .o1(\s[4] ));
  aoi012aa1n02x5               g226(.a(new_n117), .b(\a[5] ), .c(\b[4] ), .o1(new_n322));
  xnrc02aa1n02x5               g227(.a(\b[4] ), .b(\a[5] ), .out0(new_n323));
  nanp02aa1n02x5               g228(.a(new_n114), .b(new_n116), .o1(new_n324));
  aoi022aa1n02x5               g229(.a(new_n324), .b(new_n323), .c(new_n114), .d(new_n322), .o1(\s[5] ));
  and002aa1n02x5               g230(.a(new_n114), .b(new_n322), .o(new_n326));
  inv000aa1n03x5               g231(.a(new_n326), .o1(new_n327));
  xobna2aa1n03x5               g232(.a(new_n115), .b(new_n327), .c(new_n99), .out0(\s[6] ));
  inv000aa1d42x5               g233(.a(new_n100), .o1(new_n329));
  nanp02aa1n02x5               g234(.a(new_n114), .b(new_n118), .o1(new_n330));
  xobna2aa1n03x5               g235(.a(new_n102), .b(new_n330), .c(new_n329), .out0(\s[7] ));
  tech160nm_fiao0012aa1n02p5x5 g236(.a(new_n102), .b(new_n330), .c(new_n329), .o(new_n332));
  xobna2aa1n03x5               g237(.a(new_n101), .b(new_n332), .c(new_n105), .out0(\s[8] ));
  xnbna2aa1n03x5               g238(.a(new_n121), .b(new_n119), .c(new_n107), .out0(\s[9] ));
endmodule


