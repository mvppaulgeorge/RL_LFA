// Benchmark "adder" written by ABC on Wed Jul 17 23:33:07 2024

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
    new_n126, new_n127, new_n128, new_n130, new_n131, new_n132, new_n133,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n150, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n307, new_n309, new_n310,
    new_n312, new_n314, new_n316, new_n318, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\b[9] ), .o1(new_n97));
  nanb02aa1d24x5               g002(.a(\a[10] ), .b(new_n97), .out0(new_n98));
  nand42aa1n10x5               g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  xorc02aa1n02x5               g005(.a(\a[3] ), .b(\b[2] ), .out0(new_n101));
  inv040aa1d30x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nand02aa1d08x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  oao003aa1n03x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .carry(new_n105));
  nanp02aa1n04x5               g010(.a(new_n105), .b(new_n101), .o1(new_n106));
  oa0022aa1n09x5               g011(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n107));
  nor022aa1n16x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  aoi122aa1n06x5               g013(.a(new_n108), .b(\b[6] ), .c(\a[7] ), .d(\b[4] ), .e(\a[5] ), .o1(new_n109));
  tech160nm_fixorc02aa1n05x5   g014(.a(\a[8] ), .b(\b[7] ), .out0(new_n110));
  nor042aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1d28x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  oai012aa1n03x5               g017(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .o1(new_n113));
  aoi112aa1n03x5               g018(.a(new_n113), .b(new_n111), .c(\a[6] ), .d(\b[5] ), .o1(new_n114));
  nand03aa1n02x5               g019(.a(new_n114), .b(new_n109), .c(new_n110), .o1(new_n115));
  nor002aa1n06x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  aoi022aa1d24x5               g021(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n117));
  oai012aa1n12x5               g022(.a(new_n117), .b(new_n116), .c(new_n108), .o1(new_n118));
  oab012aa1n12x5               g023(.a(new_n111), .b(\a[8] ), .c(\b[7] ), .out0(new_n119));
  aoi022aa1n12x5               g024(.a(new_n118), .b(new_n119), .c(\b[7] ), .d(\a[8] ), .o1(new_n120));
  inv030aa1n02x5               g025(.a(new_n120), .o1(new_n121));
  aoai13aa1n06x5               g026(.a(new_n121), .b(new_n115), .c(new_n106), .d(new_n107), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  aoi012aa1n06x5               g028(.a(new_n100), .b(new_n122), .c(new_n123), .o1(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n124), .b(new_n98), .c(new_n99), .out0(\s[10] ));
  nanp02aa1n02x5               g030(.a(new_n98), .b(new_n99), .o1(new_n126));
  nanb02aa1n06x5               g031(.a(new_n126), .b(new_n124), .out0(new_n127));
  xorc02aa1n02x5               g032(.a(\a[11] ), .b(\b[10] ), .out0(new_n128));
  xobna2aa1n03x5               g033(.a(new_n128), .b(new_n127), .c(new_n99), .out0(\s[11] ));
  orn002aa1n03x5               g034(.a(\a[11] ), .b(\b[10] ), .o(new_n130));
  nand23aa1n04x5               g035(.a(new_n127), .b(new_n99), .c(new_n128), .o1(new_n131));
  tech160nm_fixnrc02aa1n05x5   g036(.a(\b[11] ), .b(\a[12] ), .out0(new_n132));
  tech160nm_fiaoi012aa1n02p5x5 g037(.a(new_n132), .b(new_n131), .c(new_n130), .o1(new_n133));
  nand23aa1n03x5               g038(.a(new_n131), .b(new_n130), .c(new_n132), .o1(new_n134));
  norb02aa1n03x4               g039(.a(new_n134), .b(new_n133), .out0(\s[12] ));
  oai112aa1n06x5               g040(.a(new_n98), .b(new_n99), .c(\b[8] ), .d(\a[9] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  and003aa1n03x5               g042(.a(new_n130), .b(new_n123), .c(new_n137), .o(new_n138));
  nona22aa1n03x5               g043(.a(new_n138), .b(new_n136), .c(new_n132), .out0(new_n139));
  nanb02aa1n06x5               g044(.a(new_n139), .b(new_n122), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  aoi022aa1d24x5               g046(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n142));
  oai022aa1n02x5               g047(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n143));
  aoai13aa1n06x5               g048(.a(new_n141), .b(new_n143), .c(new_n136), .d(new_n142), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(new_n140), .b(new_n144), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g051(.a(\a[14] ), .o1(new_n147));
  inv000aa1d42x5               g052(.a(\a[13] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\b[12] ), .o1(new_n149));
  oaoi03aa1n02x5               g054(.a(new_n148), .b(new_n149), .c(new_n145), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[13] ), .c(new_n147), .out0(\s[14] ));
  xroi22aa1d04x5               g056(.a(new_n148), .b(\b[12] ), .c(new_n147), .d(\b[13] ), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nor002aa1d32x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nand02aa1d08x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n154), .c(new_n148), .d(new_n149), .o1(new_n156));
  aoai13aa1n04x5               g061(.a(new_n156), .b(new_n153), .c(new_n140), .d(new_n144), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  and002aa1n03x5               g064(.a(\b[14] ), .b(\a[15] ), .o(new_n160));
  norp02aa1n02x5               g065(.a(new_n160), .b(new_n159), .o1(new_n161));
  xorc02aa1n12x5               g066(.a(\a[16] ), .b(\b[15] ), .out0(new_n162));
  aoi112aa1n02x5               g067(.a(new_n159), .b(new_n162), .c(new_n157), .d(new_n161), .o1(new_n163));
  aoai13aa1n03x5               g068(.a(new_n162), .b(new_n159), .c(new_n157), .d(new_n161), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(\s[16] ));
  xnrc02aa1n02x5               g070(.a(\b[2] ), .b(\a[3] ), .out0(new_n166));
  tech160nm_fioaoi03aa1n02p5x5 g071(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n167));
  tech160nm_fioai012aa1n05x5   g072(.a(new_n107), .b(new_n167), .c(new_n166), .o1(new_n168));
  and003aa1n03x7               g073(.a(new_n114), .b(new_n110), .c(new_n109), .o(new_n169));
  nor002aa1n12x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  norb03aa1n03x5               g075(.a(new_n155), .b(new_n170), .c(new_n154), .out0(new_n171));
  aoi112aa1n06x5               g076(.a(new_n160), .b(new_n159), .c(\a[13] ), .d(\b[12] ), .o1(new_n172));
  nand23aa1n09x5               g077(.a(new_n172), .b(new_n162), .c(new_n171), .o1(new_n173));
  nor042aa1n06x5               g078(.a(new_n139), .b(new_n173), .o1(new_n174));
  aoai13aa1n12x5               g079(.a(new_n174), .b(new_n120), .c(new_n169), .d(new_n168), .o1(new_n175));
  oaoi13aa1n09x5               g080(.a(new_n159), .b(new_n155), .c(new_n170), .d(new_n154), .o1(new_n176));
  ao0022aa1n03x5               g081(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o(new_n177));
  oai022aa1n09x5               g082(.a(new_n176), .b(new_n177), .c(\b[15] ), .d(\a[16] ), .o1(new_n178));
  oab012aa1n12x5               g083(.a(new_n178), .b(new_n144), .c(new_n173), .out0(new_n179));
  xorc02aa1n02x5               g084(.a(\a[17] ), .b(\b[16] ), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n175), .c(new_n179), .out0(\s[17] ));
  inv040aa1d32x5               g086(.a(\a[17] ), .o1(new_n182));
  inv040aa1d32x5               g087(.a(\b[16] ), .o1(new_n183));
  nand42aa1n02x5               g088(.a(new_n183), .b(new_n182), .o1(new_n184));
  oabi12aa1n06x5               g089(.a(new_n178), .b(new_n144), .c(new_n173), .out0(new_n185));
  aoai13aa1n02x5               g090(.a(new_n180), .b(new_n185), .c(new_n122), .d(new_n174), .o1(new_n186));
  nor002aa1d32x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nand42aa1d28x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  nanb02aa1n12x5               g093(.a(new_n187), .b(new_n188), .out0(new_n189));
  xobna2aa1n03x5               g094(.a(new_n189), .b(new_n186), .c(new_n184), .out0(\s[18] ));
  nanp02aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  nano22aa1n12x5               g096(.a(new_n189), .b(new_n184), .c(new_n191), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n188), .b(new_n187), .c(new_n182), .d(new_n183), .o1(new_n194));
  aoai13aa1n04x5               g099(.a(new_n194), .b(new_n193), .c(new_n175), .d(new_n179), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n12x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nand02aa1n08x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nor022aa1n16x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand22aa1n09x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aoi112aa1n02x5               g107(.a(new_n198), .b(new_n202), .c(new_n195), .d(new_n199), .o1(new_n203));
  aoai13aa1n03x5               g108(.a(new_n202), .b(new_n198), .c(new_n195), .d(new_n199), .o1(new_n204));
  norb02aa1n02x7               g109(.a(new_n204), .b(new_n203), .out0(\s[20] ));
  nano23aa1n09x5               g110(.a(new_n198), .b(new_n200), .c(new_n201), .d(new_n199), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n192), .b(new_n206), .o1(new_n207));
  nona23aa1d18x5               g112(.a(new_n201), .b(new_n199), .c(new_n198), .d(new_n200), .out0(new_n208));
  aoi012aa1n12x5               g113(.a(new_n200), .b(new_n198), .c(new_n201), .o1(new_n209));
  oai012aa1d24x5               g114(.a(new_n209), .b(new_n208), .c(new_n194), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n04x5               g116(.a(new_n211), .b(new_n207), .c(new_n175), .d(new_n179), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[21] ), .b(\b[20] ), .out0(new_n215));
  xorc02aa1n02x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  aoi112aa1n02x5               g121(.a(new_n214), .b(new_n216), .c(new_n212), .d(new_n215), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n216), .b(new_n214), .c(new_n212), .d(new_n215), .o1(new_n218));
  norb02aa1n02x7               g123(.a(new_n218), .b(new_n217), .out0(\s[22] ));
  inv000aa1d42x5               g124(.a(\a[21] ), .o1(new_n220));
  inv040aa1d32x5               g125(.a(\a[22] ), .o1(new_n221));
  xroi22aa1d06x4               g126(.a(new_n220), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n222));
  nand23aa1n02x5               g127(.a(new_n222), .b(new_n192), .c(new_n206), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\b[21] ), .o1(new_n224));
  tech160nm_fioaoi03aa1n03p5x5 g129(.a(new_n221), .b(new_n224), .c(new_n214), .o1(new_n225));
  inv000aa1n02x5               g130(.a(new_n225), .o1(new_n226));
  aoi012aa1n02x5               g131(.a(new_n226), .b(new_n210), .c(new_n222), .o1(new_n227));
  aoai13aa1n04x5               g132(.a(new_n227), .b(new_n223), .c(new_n175), .d(new_n179), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g134(.a(\b[22] ), .b(\a[23] ), .o1(new_n230));
  xorc02aa1n03x5               g135(.a(\a[23] ), .b(\b[22] ), .out0(new_n231));
  xorc02aa1n03x5               g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  aoi112aa1n02x7               g137(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n234));
  norb02aa1n02x7               g139(.a(new_n234), .b(new_n233), .out0(\s[24] ));
  nanp02aa1n06x5               g140(.a(new_n175), .b(new_n179), .o1(new_n236));
  and002aa1n06x5               g141(.a(new_n232), .b(new_n231), .o(new_n237));
  inv040aa1n02x5               g142(.a(new_n237), .o1(new_n238));
  nano32aa1n02x5               g143(.a(new_n238), .b(new_n222), .c(new_n206), .d(new_n192), .out0(new_n239));
  inv000aa1n03x5               g144(.a(new_n194), .o1(new_n240));
  inv020aa1n02x5               g145(.a(new_n209), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n222), .b(new_n241), .c(new_n206), .d(new_n240), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n243));
  oab012aa1n02x4               g148(.a(new_n243), .b(\a[24] ), .c(\b[23] ), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n238), .c(new_n242), .d(new_n225), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n236), .c(new_n239), .o1(new_n246));
  xorc02aa1n12x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  xnrc02aa1n02x5               g152(.a(new_n246), .b(new_n247), .out0(\s[25] ));
  nor042aa1n03x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n247), .b(new_n245), .c(new_n236), .d(new_n239), .o1(new_n250));
  xorc02aa1n12x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  nona22aa1n02x5               g156(.a(new_n250), .b(new_n251), .c(new_n249), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n249), .o1(new_n253));
  aobi12aa1n06x5               g158(.a(new_n251), .b(new_n250), .c(new_n253), .out0(new_n254));
  norb02aa1n03x4               g159(.a(new_n252), .b(new_n254), .out0(\s[26] ));
  and002aa1n12x5               g160(.a(new_n251), .b(new_n247), .o(new_n256));
  nano22aa1n03x7               g161(.a(new_n223), .b(new_n237), .c(new_n256), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n185), .c(new_n122), .d(new_n174), .o1(new_n258));
  nand22aa1n03x5               g163(.a(new_n245), .b(new_n256), .o1(new_n259));
  oao003aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n260));
  nor042aa1n04x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  norb02aa1n15x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  aoi013aa1n03x5               g169(.a(new_n264), .b(new_n259), .c(new_n258), .d(new_n260), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n237), .b(new_n226), .c(new_n210), .d(new_n222), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n256), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n260), .b(new_n267), .c(new_n266), .d(new_n244), .o1(new_n268));
  nano22aa1n02x4               g173(.a(new_n268), .b(new_n258), .c(new_n264), .out0(new_n269));
  norp02aa1n02x5               g174(.a(new_n265), .b(new_n269), .o1(\s[27] ));
  inv000aa1n06x5               g175(.a(new_n261), .o1(new_n271));
  xnrc02aa1n12x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  nano22aa1n03x5               g177(.a(new_n265), .b(new_n271), .c(new_n272), .out0(new_n273));
  inv000aa1n02x5               g178(.a(new_n257), .o1(new_n274));
  aoi012aa1n06x5               g179(.a(new_n274), .b(new_n175), .c(new_n179), .o1(new_n275));
  oaih12aa1n02x5               g180(.a(new_n263), .b(new_n268), .c(new_n275), .o1(new_n276));
  tech160nm_fiaoi012aa1n02p5x5 g181(.a(new_n272), .b(new_n276), .c(new_n271), .o1(new_n277));
  norp02aa1n03x5               g182(.a(new_n277), .b(new_n273), .o1(\s[28] ));
  nano22aa1n12x5               g183(.a(new_n272), .b(new_n271), .c(new_n262), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n279), .b(new_n268), .c(new_n275), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n279), .o1(new_n284));
  aoi013aa1n03x5               g189(.a(new_n284), .b(new_n259), .c(new_n258), .d(new_n260), .o1(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n281), .c(new_n282), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n283), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g193(.a(new_n282), .b(new_n272), .c(new_n262), .d(new_n271), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n289), .b(new_n268), .c(new_n275), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n289), .o1(new_n294));
  aoi013aa1n02x5               g199(.a(new_n294), .b(new_n259), .c(new_n258), .d(new_n260), .o1(new_n295));
  nano22aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n292), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n293), .b(new_n296), .o1(\s[30] ));
  norb03aa1n03x4               g202(.a(new_n279), .b(new_n292), .c(new_n282), .out0(new_n298));
  inv000aa1n02x5               g203(.a(new_n298), .o1(new_n299));
  aoi013aa1n03x5               g204(.a(new_n299), .b(new_n259), .c(new_n258), .d(new_n260), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n298), .b(new_n268), .c(new_n275), .o1(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n03x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  inv000aa1d42x5               g211(.a(\a[3] ), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n167), .b(\b[2] ), .c(new_n307), .out0(\s[3] ));
  xorc02aa1n02x5               g213(.a(\a[4] ), .b(\b[3] ), .out0(new_n309));
  aoib12aa1n02x5               g214(.a(new_n309), .b(new_n307), .c(\b[2] ), .out0(new_n310));
  aoi022aa1n02x5               g215(.a(new_n168), .b(new_n309), .c(new_n106), .d(new_n310), .o1(\s[4] ));
  xorc02aa1n02x5               g216(.a(\a[5] ), .b(\b[4] ), .out0(new_n312));
  xobna2aa1n03x5               g217(.a(new_n312), .b(new_n168), .c(new_n112), .out0(\s[5] ));
  aoi013aa1n02x4               g218(.a(new_n116), .b(new_n168), .c(new_n112), .d(new_n312), .o1(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g220(.a(\a[6] ), .b(\b[5] ), .c(new_n314), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanp02aa1n02x5               g222(.a(\b[6] ), .b(\a[7] ), .o1(new_n318));
  aoi012aa1n02x5               g223(.a(new_n111), .b(new_n316), .c(new_n318), .o1(new_n319));
  xnrc02aa1n02x5               g224(.a(new_n319), .b(new_n110), .out0(\s[8] ));
  xorb03aa1n02x5               g225(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


