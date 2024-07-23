// Benchmark "adder" written by ABC on Wed Jul 17 16:42:29 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n309, new_n311, new_n312, new_n313,
    new_n314, new_n316, new_n317, new_n318, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor002aa1d32x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor022aa1n16x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  tech160nm_finand02aa1n03p5x5 g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n09x5               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  nor043aa1n04x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  tech160nm_fiaoi012aa1n05x5   g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  norp02aa1n06x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nand22aa1n12x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  tech160nm_fiao0012aa1n02p5x5 g019(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n115));
  oabi12aa1n09x5               g020(.a(new_n115), .b(new_n114), .c(new_n109), .out0(new_n116));
  inv000aa1d42x5               g021(.a(new_n98), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n100), .b(new_n99), .o1(new_n118));
  inv040aa1d28x5               g023(.a(\b[5] ), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  oaib12aa1n02x7               g025(.a(new_n120), .b(new_n119), .c(\a[6] ), .out0(new_n121));
  oai112aa1n06x5               g026(.a(new_n117), .b(new_n118), .c(new_n102), .d(new_n121), .o1(new_n122));
  tech160nm_fixorc02aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n03x5               g028(.a(new_n123), .b(new_n122), .c(new_n116), .d(new_n105), .o1(new_n124));
  tech160nm_fixorc02aa1n03p5x5 g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  and002aa1n12x5               g031(.a(\b[9] ), .b(\a[10] ), .o(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  nor002aa1n04x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n04x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n03x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  oai112aa1n03x5               g037(.a(new_n124), .b(new_n97), .c(\b[9] ), .d(\a[10] ), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n132), .b(new_n133), .c(new_n128), .out0(\s[11] ));
  aoi013aa1n03x5               g039(.a(new_n129), .b(new_n133), .c(new_n131), .d(new_n128), .o1(new_n135));
  nor042aa1n09x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  nanp02aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n135), .b(new_n138), .c(new_n137), .out0(\s[12] ));
  nano23aa1n09x5               g044(.a(new_n129), .b(new_n136), .c(new_n138), .d(new_n130), .out0(new_n140));
  and003aa1n02x5               g045(.a(new_n140), .b(new_n125), .c(new_n123), .o(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n122), .c(new_n116), .d(new_n105), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n143));
  oa0022aa1n02x5               g048(.a(\b[9] ), .b(\a[10] ), .c(\b[8] ), .d(\a[9] ), .o(new_n144));
  nona22aa1n09x5               g049(.a(new_n140), .b(new_n144), .c(new_n127), .out0(new_n145));
  nona22aa1d18x5               g050(.a(new_n145), .b(new_n143), .c(new_n136), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  nor042aa1n04x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  nand42aa1n20x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  xnbna2aa1n03x5               g055(.a(new_n150), .b(new_n142), .c(new_n147), .out0(\s[13] ));
  inv000aa1d42x5               g056(.a(\a[13] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(\b[12] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n142), .b(new_n147), .o1(new_n154));
  oaoi03aa1n02x5               g059(.a(new_n152), .b(new_n153), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nand42aa1n06x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  nor042aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand02aa1n16x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1d15x5               g066(.a(new_n148), .b(new_n160), .c(new_n161), .d(new_n149), .out0(new_n162));
  oa0012aa1n06x5               g067(.a(new_n161), .b(new_n160), .c(new_n148), .o(new_n163));
  aoai13aa1n04x5               g068(.a(new_n159), .b(new_n163), .c(new_n154), .d(new_n162), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n159), .b(new_n163), .c(new_n154), .d(new_n162), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(\s[15] ));
  nor042aa1n04x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nand42aa1n06x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nona22aa1n02x4               g074(.a(new_n164), .b(new_n169), .c(new_n157), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n169), .o1(new_n171));
  oaoi13aa1n02x5               g076(.a(new_n171), .b(new_n164), .c(\a[15] ), .d(\b[14] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n170), .b(new_n172), .out0(\s[16] ));
  nano23aa1n09x5               g078(.a(new_n157), .b(new_n167), .c(new_n168), .d(new_n158), .out0(new_n174));
  nand22aa1n09x5               g079(.a(new_n174), .b(new_n162), .o1(new_n175));
  nano32aa1n03x7               g080(.a(new_n175), .b(new_n140), .c(new_n125), .d(new_n123), .out0(new_n176));
  aoai13aa1n12x5               g081(.a(new_n176), .b(new_n122), .c(new_n116), .d(new_n105), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n175), .o1(new_n178));
  aoi112aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n179));
  nanp02aa1n03x5               g084(.a(new_n174), .b(new_n163), .o1(new_n180));
  nona22aa1n03x5               g085(.a(new_n180), .b(new_n179), .c(new_n167), .out0(new_n181));
  aoi012aa1d24x5               g086(.a(new_n181), .b(new_n146), .c(new_n178), .o1(new_n182));
  nanp02aa1n12x5               g087(.a(new_n182), .b(new_n177), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n03x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d06x4               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  oai022aa1n04x7               g095(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n191));
  oaib12aa1n12x5               g096(.a(new_n191), .b(new_n185), .c(\b[17] ), .out0(new_n192));
  inv040aa1n03x5               g097(.a(new_n192), .o1(new_n193));
  nor042aa1n04x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nand22aa1n03x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand22aa1n04x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n02x5               g108(.a(new_n197), .b(new_n203), .c(new_n194), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n203), .o1(new_n205));
  oaoi13aa1n02x7               g110(.a(new_n205), .b(new_n197), .c(\a[19] ), .d(\b[18] ), .o1(new_n206));
  norb02aa1n03x4               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n06x5               g112(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n190), .b(new_n208), .o1(new_n209));
  nona23aa1n09x5               g114(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n210));
  aoi012aa1n12x5               g115(.a(new_n201), .b(new_n194), .c(new_n202), .o1(new_n211));
  oai012aa1d24x5               g116(.a(new_n211), .b(new_n210), .c(new_n192), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n04x5               g118(.a(new_n213), .b(new_n209), .c(new_n182), .d(new_n177), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x7               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n218), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  norb02aa1n02x7               g125(.a(new_n220), .b(new_n219), .out0(\s[22] ));
  inv000aa1d42x5               g126(.a(\a[21] ), .o1(new_n222));
  inv040aa1d32x5               g127(.a(\a[22] ), .o1(new_n223));
  xroi22aa1d06x4               g128(.a(new_n222), .b(\b[20] ), .c(new_n223), .d(\b[21] ), .out0(new_n224));
  nanp03aa1n02x5               g129(.a(new_n224), .b(new_n190), .c(new_n208), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[21] ), .o1(new_n226));
  oaoi03aa1n12x5               g131(.a(new_n223), .b(new_n226), .c(new_n216), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n212), .c(new_n224), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n225), .c(new_n182), .d(new_n177), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x7               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  and002aa1n02x5               g142(.a(new_n234), .b(new_n233), .o(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  nano32aa1n02x4               g144(.a(new_n239), .b(new_n224), .c(new_n190), .d(new_n208), .out0(new_n240));
  inv000aa1n02x5               g145(.a(new_n211), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n224), .b(new_n241), .c(new_n208), .d(new_n193), .o1(new_n242));
  orn002aa1n02x5               g147(.a(\a[23] ), .b(\b[22] ), .o(new_n243));
  oao003aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .c(new_n243), .carry(new_n244));
  aoai13aa1n12x5               g149(.a(new_n244), .b(new_n239), .c(new_n242), .d(new_n227), .o1(new_n245));
  xorc02aa1n12x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n245), .c(new_n183), .d(new_n240), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n246), .b(new_n245), .c(new_n183), .d(new_n240), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[25] ));
  nor042aa1n03x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n12x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  nona22aa1n03x5               g156(.a(new_n247), .b(new_n251), .c(new_n250), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n250), .o1(new_n253));
  aobi12aa1n06x5               g158(.a(new_n251), .b(new_n247), .c(new_n253), .out0(new_n254));
  norb02aa1n03x4               g159(.a(new_n252), .b(new_n254), .out0(\s[26] ));
  and002aa1n06x5               g160(.a(new_n251), .b(new_n246), .o(new_n256));
  nano22aa1n03x7               g161(.a(new_n225), .b(new_n238), .c(new_n256), .out0(new_n257));
  nand02aa1d10x5               g162(.a(new_n183), .b(new_n257), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n259));
  aobi12aa1n06x5               g164(.a(new_n259), .b(new_n245), .c(new_n256), .out0(new_n260));
  xorc02aa1n12x5               g165(.a(\a[27] ), .b(\b[26] ), .out0(new_n261));
  xnbna2aa1n06x5               g166(.a(new_n261), .b(new_n260), .c(new_n258), .out0(\s[27] ));
  nor042aa1n03x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n261), .o1(new_n265));
  aoi012aa1n06x5               g170(.a(new_n265), .b(new_n260), .c(new_n258), .o1(new_n266));
  tech160nm_fixnrc02aa1n05x5   g171(.a(\b[27] ), .b(\a[28] ), .out0(new_n267));
  nano22aa1n03x7               g172(.a(new_n266), .b(new_n264), .c(new_n267), .out0(new_n268));
  aobi12aa1n06x5               g173(.a(new_n257), .b(new_n182), .c(new_n177), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n238), .b(new_n228), .c(new_n212), .d(new_n224), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n256), .o1(new_n271));
  aoai13aa1n04x5               g176(.a(new_n259), .b(new_n271), .c(new_n270), .d(new_n244), .o1(new_n272));
  oaih12aa1n02x5               g177(.a(new_n261), .b(new_n272), .c(new_n269), .o1(new_n273));
  tech160nm_fiaoi012aa1n02p5x5 g178(.a(new_n267), .b(new_n273), .c(new_n264), .o1(new_n274));
  nor002aa1n02x5               g179(.a(new_n274), .b(new_n268), .o1(\s[28] ));
  norb02aa1n15x5               g180(.a(new_n261), .b(new_n267), .out0(new_n276));
  tech160nm_fioai012aa1n03p5x5 g181(.a(new_n276), .b(new_n272), .c(new_n269), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .c(new_n264), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  tech160nm_fiaoi012aa1n05x5   g184(.a(new_n279), .b(new_n277), .c(new_n278), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n276), .o1(new_n281));
  aoi012aa1n06x5               g186(.a(new_n281), .b(new_n260), .c(new_n258), .o1(new_n282));
  nano22aa1n03x7               g187(.a(new_n282), .b(new_n278), .c(new_n279), .out0(new_n283));
  norp02aa1n03x5               g188(.a(new_n280), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g190(.a(new_n261), .b(new_n279), .c(new_n267), .out0(new_n286));
  oaih12aa1n02x5               g191(.a(new_n286), .b(new_n272), .c(new_n269), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n278), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n286), .o1(new_n291));
  aoi012aa1n06x5               g196(.a(new_n291), .b(new_n260), .c(new_n258), .o1(new_n292));
  nano22aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n289), .out0(new_n293));
  nor002aa1n02x5               g198(.a(new_n290), .b(new_n293), .o1(\s[30] ));
  xnrc02aa1n02x5               g199(.a(\b[30] ), .b(\a[31] ), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n286), .b(new_n289), .out0(new_n296));
  inv000aa1n02x5               g201(.a(new_n296), .o1(new_n297));
  aoi012aa1n06x5               g202(.a(new_n297), .b(new_n260), .c(new_n258), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n299));
  nano22aa1n03x5               g204(.a(new_n298), .b(new_n295), .c(new_n299), .out0(new_n300));
  tech160nm_fioai012aa1n03p5x5 g205(.a(new_n296), .b(new_n272), .c(new_n269), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n295), .b(new_n301), .c(new_n299), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n300), .o1(\s[31] ));
  xnrb03aa1n02x5               g208(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g209(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g211(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g212(.a(new_n104), .b(new_n116), .out0(new_n308));
  tech160nm_fioai012aa1n03p5x5 g213(.a(new_n308), .b(\b[4] ), .c(\a[5] ), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g215(.a(new_n100), .b(new_n101), .out0(new_n311));
  nanb02aa1n02x5               g216(.a(new_n103), .b(new_n309), .out0(new_n312));
  oaoi13aa1n06x5               g217(.a(new_n311), .b(new_n312), .c(\a[6] ), .d(\b[5] ), .o1(new_n313));
  oai112aa1n02x5               g218(.a(new_n312), .b(new_n311), .c(\b[5] ), .d(\a[6] ), .o1(new_n314));
  norb02aa1n02x5               g219(.a(new_n314), .b(new_n313), .out0(\s[7] ));
  nanb02aa1n02x5               g220(.a(new_n98), .b(new_n99), .out0(new_n316));
  oab012aa1n02x4               g221(.a(new_n316), .b(new_n313), .c(new_n100), .out0(new_n317));
  aoi112aa1n02x5               g222(.a(new_n313), .b(new_n100), .c(new_n117), .d(new_n99), .o1(new_n318));
  norp02aa1n02x5               g223(.a(new_n317), .b(new_n318), .o1(\s[8] ));
  aoi112aa1n02x5               g224(.a(new_n122), .b(new_n123), .c(new_n116), .d(new_n105), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n124), .b(new_n320), .out0(\s[9] ));
endmodule


