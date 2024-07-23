// Benchmark "adder" written by ABC on Thu Jul 18 05:12:22 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n128, new_n129, new_n130, new_n131, new_n132, new_n133,
    new_n135, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n149, new_n150,
    new_n151, new_n152, new_n153, new_n154, new_n156, new_n157, new_n158,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n208, new_n209, new_n210, new_n211, new_n212, new_n213, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n307, new_n309, new_n310,
    new_n312, new_n313, new_n316, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\b[8] ), .o1(new_n97));
  tech160nm_fixnrc02aa1n04x5   g002(.a(\b[2] ), .b(\a[3] ), .out0(new_n98));
  inv000aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[2] ), .o1(new_n100));
  inv040aa1d32x5               g005(.a(\b[1] ), .o1(new_n101));
  nand02aa1d16x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  oao003aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .carry(new_n103));
  tech160nm_finand02aa1n03p5x5 g008(.a(new_n99), .b(new_n103), .o1(new_n104));
  oa0022aa1n02x5               g009(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n105));
  nand42aa1n03x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor002aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nand42aa1n03x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nano22aa1n03x7               g013(.a(new_n107), .b(new_n106), .c(new_n108), .out0(new_n109));
  tech160nm_fixnrc02aa1n05x5   g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  norp02aa1n24x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n24x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1d28x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1d15x5               g019(.a(new_n114), .b(new_n111), .c(new_n112), .d(new_n113), .out0(new_n115));
  nanb03aa1n12x5               g020(.a(new_n110), .b(new_n115), .c(new_n109), .out0(new_n116));
  orn002aa1n24x5               g021(.a(\a[5] ), .b(\b[4] ), .o(new_n117));
  oaoi03aa1n09x5               g022(.a(\a[6] ), .b(\b[5] ), .c(new_n117), .o1(new_n118));
  oai012aa1n02x7               g023(.a(new_n112), .b(new_n114), .c(new_n111), .o1(new_n119));
  aobi12aa1n12x5               g024(.a(new_n119), .b(new_n115), .c(new_n118), .out0(new_n120));
  aoai13aa1n12x5               g025(.a(new_n120), .b(new_n116), .c(new_n104), .d(new_n105), .o1(new_n121));
  oaib12aa1n09x5               g026(.a(new_n121), .b(new_n97), .c(\a[9] ), .out0(new_n122));
  oa0012aa1n02x5               g027(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .o(new_n123));
  xnrb03aa1n03x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oaih22aa1n04x5               g029(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n125));
  aboi22aa1n12x5               g030(.a(new_n125), .b(new_n122), .c(\b[9] ), .d(\a[10] ), .out0(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nand42aa1n04x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nor022aa1n16x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  xorc02aa1n12x5               g034(.a(\a[12] ), .b(\b[11] ), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  aoai13aa1n02x7               g036(.a(new_n131), .b(new_n129), .c(new_n126), .d(new_n128), .o1(new_n132));
  aoi112aa1n03x5               g037(.a(new_n131), .b(new_n129), .c(new_n126), .d(new_n128), .o1(new_n133));
  nanb02aa1n03x5               g038(.a(new_n133), .b(new_n132), .out0(\s[12] ));
  nanp02aa1n04x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  norp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  nano23aa1n06x5               g042(.a(new_n129), .b(new_n136), .c(new_n137), .d(new_n128), .out0(new_n138));
  xorc02aa1n12x5               g043(.a(\a[9] ), .b(\b[8] ), .out0(new_n139));
  nand23aa1d12x5               g044(.a(new_n138), .b(new_n130), .c(new_n139), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  aoi022aa1n02x7               g046(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n142));
  oab012aa1n02x4               g047(.a(new_n129), .b(\a[12] ), .c(\b[11] ), .out0(new_n143));
  aob012aa1n06x5               g048(.a(new_n143), .b(new_n125), .c(new_n142), .out0(new_n144));
  aoi022aa1n09x5               g049(.a(new_n121), .b(new_n141), .c(new_n135), .d(new_n144), .o1(new_n145));
  xnrb03aa1n02x5               g050(.a(new_n145), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  oaoi03aa1n03x5               g051(.a(\a[13] ), .b(\b[12] ), .c(new_n145), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .out0(new_n149));
  tech160nm_fixnrc02aa1n02p5x5 g054(.a(\b[13] ), .b(\a[14] ), .out0(new_n150));
  norp02aa1n02x5               g055(.a(new_n150), .b(new_n149), .o1(new_n151));
  oai022aa1d18x5               g056(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n152));
  aob012aa1n03x5               g057(.a(new_n152), .b(\b[13] ), .c(\a[14] ), .out0(new_n153));
  oaib12aa1n06x5               g058(.a(new_n153), .b(new_n145), .c(new_n151), .out0(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanp02aa1n04x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nor002aa1d32x5               g062(.a(\b[15] ), .b(\a[16] ), .o1(new_n158));
  nanp02aa1n04x5               g063(.a(\b[15] ), .b(\a[16] ), .o1(new_n159));
  nanb02aa1n03x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  aoai13aa1n03x5               g065(.a(new_n160), .b(new_n156), .c(new_n154), .d(new_n157), .o1(new_n161));
  inv000aa1n02x5               g066(.a(new_n157), .o1(new_n162));
  nona22aa1n03x5               g067(.a(new_n154), .b(new_n156), .c(new_n162), .out0(new_n163));
  nona22aa1n03x5               g068(.a(new_n163), .b(new_n160), .c(new_n156), .out0(new_n164));
  nanp02aa1n03x5               g069(.a(new_n164), .b(new_n161), .o1(\s[16] ));
  inv000aa1d42x5               g070(.a(\a[17] ), .o1(new_n166));
  nano23aa1n02x4               g071(.a(new_n156), .b(new_n158), .c(new_n159), .d(new_n157), .out0(new_n167));
  nano22aa1n12x5               g072(.a(new_n140), .b(new_n151), .c(new_n167), .out0(new_n168));
  aoi112aa1n03x5               g073(.a(new_n162), .b(new_n156), .c(\a[13] ), .d(\b[12] ), .o1(new_n169));
  oai012aa1n02x5               g074(.a(new_n135), .b(\b[12] ), .c(\a[13] ), .o1(new_n170));
  nor043aa1n03x5               g075(.a(new_n150), .b(new_n160), .c(new_n170), .o1(new_n171));
  inv000aa1n02x5               g076(.a(new_n156), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n158), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(new_n159), .b(new_n157), .o1(new_n174));
  aoai13aa1n02x7               g079(.a(new_n173), .b(new_n174), .c(new_n153), .d(new_n172), .o1(new_n175));
  aoi013aa1n06x4               g080(.a(new_n175), .b(new_n171), .c(new_n144), .d(new_n169), .o1(new_n176));
  inv030aa1n04x5               g081(.a(new_n176), .o1(new_n177));
  aoi012aa1n09x5               g082(.a(new_n177), .b(new_n121), .c(new_n168), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(new_n166), .out0(\s[17] ));
  oaoi03aa1n03x5               g084(.a(\a[17] ), .b(\b[16] ), .c(new_n178), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv040aa1d32x5               g086(.a(\a[18] ), .o1(new_n182));
  xroi22aa1d04x5               g087(.a(new_n166), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n183));
  norp02aa1n02x5               g088(.a(\b[17] ), .b(\a[18] ), .o1(new_n184));
  aoi112aa1n03x5               g089(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n185));
  nor002aa1n02x5               g090(.a(new_n185), .b(new_n184), .o1(new_n186));
  oaib12aa1n06x5               g091(.a(new_n186), .b(new_n178), .c(new_n183), .out0(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g093(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nand22aa1n04x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  norb02aa1n06x4               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  nor002aa1n16x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nand02aa1d16x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  norb02aa1n12x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  aoai13aa1n03x5               g101(.a(new_n196), .b(new_n190), .c(new_n187), .d(new_n192), .o1(new_n197));
  nand42aa1n02x5               g102(.a(new_n187), .b(new_n192), .o1(new_n198));
  nona22aa1n03x5               g103(.a(new_n198), .b(new_n196), .c(new_n190), .out0(new_n199));
  nanp02aa1n03x5               g104(.a(new_n199), .b(new_n197), .o1(\s[20] ));
  nona23aa1n09x5               g105(.a(new_n194), .b(new_n191), .c(new_n190), .d(new_n193), .out0(new_n201));
  norb02aa1n02x5               g106(.a(new_n183), .b(new_n201), .out0(new_n202));
  inv000aa1n02x5               g107(.a(new_n202), .o1(new_n203));
  tech160nm_fioai012aa1n03p5x5 g108(.a(new_n194), .b(new_n193), .c(new_n190), .o1(new_n204));
  oaih12aa1n02x5               g109(.a(new_n204), .b(new_n201), .c(new_n186), .o1(new_n205));
  oabi12aa1n06x5               g110(.a(new_n205), .b(new_n178), .c(new_n203), .out0(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g112(.a(\b[20] ), .b(\a[21] ), .o1(new_n208));
  xorc02aa1n12x5               g113(.a(\a[21] ), .b(\b[20] ), .out0(new_n209));
  xnrc02aa1n12x5               g114(.a(\b[21] ), .b(\a[22] ), .out0(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n208), .c(new_n206), .d(new_n209), .o1(new_n211));
  nand42aa1n02x5               g116(.a(new_n206), .b(new_n209), .o1(new_n212));
  nona22aa1n03x5               g117(.a(new_n212), .b(new_n210), .c(new_n208), .out0(new_n213));
  nanp02aa1n03x5               g118(.a(new_n213), .b(new_n211), .o1(\s[22] ));
  oaoi03aa1n02x5               g119(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n215));
  oaih12aa1n02x5               g120(.a(new_n105), .b(new_n215), .c(new_n98), .o1(new_n216));
  nona23aa1n02x4               g121(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n217));
  norb03aa1n02x7               g122(.a(new_n109), .b(new_n217), .c(new_n110), .out0(new_n218));
  oaib12aa1n02x5               g123(.a(new_n119), .b(new_n217), .c(new_n118), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n168), .b(new_n219), .c(new_n216), .d(new_n218), .o1(new_n220));
  nona23aa1n06x5               g125(.a(new_n183), .b(new_n209), .c(new_n210), .d(new_n201), .out0(new_n221));
  oai112aa1n03x5               g126(.a(new_n192), .b(new_n195), .c(new_n185), .d(new_n184), .o1(new_n222));
  nanb02aa1n02x5               g127(.a(new_n210), .b(new_n209), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[22] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[21] ), .o1(new_n225));
  oao003aa1n09x5               g130(.a(new_n224), .b(new_n225), .c(new_n208), .carry(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n223), .c(new_n222), .d(new_n204), .o1(new_n228));
  inv000aa1n02x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n04x5               g134(.a(new_n229), .b(new_n221), .c(new_n220), .d(new_n176), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  tech160nm_fixorc02aa1n02p5x5 g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n12x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  aoi112aa1n03x4               g141(.a(new_n232), .b(new_n235), .c(new_n230), .d(new_n233), .o1(new_n237));
  nanb02aa1n03x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  and002aa1n06x5               g143(.a(new_n234), .b(new_n233), .o(new_n239));
  norb02aa1n03x5               g144(.a(new_n239), .b(new_n221), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n177), .c(new_n121), .d(new_n168), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n209), .b(new_n210), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n239), .b(new_n226), .c(new_n205), .d(new_n242), .o1(new_n243));
  inv000aa1d42x5               g148(.a(\a[24] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(\b[23] ), .o1(new_n245));
  oao003aa1n02x5               g150(.a(new_n244), .b(new_n245), .c(new_n232), .carry(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(new_n243), .b(new_n247), .o1(new_n248));
  nanb02aa1n06x5               g153(.a(new_n248), .b(new_n241), .out0(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  tech160nm_fixorc02aa1n02p5x5 g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  xorc02aa1n12x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n251), .c(new_n249), .d(new_n252), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n249), .b(new_n252), .o1(new_n256));
  nona22aa1n03x5               g161(.a(new_n256), .b(new_n254), .c(new_n251), .out0(new_n257));
  nanp02aa1n02x5               g162(.a(new_n257), .b(new_n255), .o1(\s[26] ));
  and002aa1n12x5               g163(.a(new_n253), .b(new_n252), .o(new_n259));
  nano22aa1n03x7               g164(.a(new_n221), .b(new_n239), .c(new_n259), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n177), .c(new_n121), .d(new_n168), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n259), .b(new_n246), .c(new_n228), .d(new_n239), .o1(new_n262));
  oai022aa1n02x5               g167(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n263));
  aob012aa1n02x5               g168(.a(new_n263), .b(\b[25] ), .c(\a[26] ), .out0(new_n264));
  nanp03aa1n03x5               g169(.a(new_n261), .b(new_n262), .c(new_n264), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  tech160nm_fixorc02aa1n04x5   g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  norp02aa1n02x5               g173(.a(\b[27] ), .b(\a[28] ), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .o1(new_n270));
  nanb02aa1n09x5               g175(.a(new_n269), .b(new_n270), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n267), .c(new_n265), .d(new_n268), .o1(new_n272));
  aobi12aa1n06x5               g177(.a(new_n260), .b(new_n220), .c(new_n176), .out0(new_n273));
  inv000aa1d42x5               g178(.a(new_n259), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n264), .b(new_n274), .c(new_n243), .d(new_n247), .o1(new_n275));
  oaih12aa1n02x5               g180(.a(new_n268), .b(new_n275), .c(new_n273), .o1(new_n276));
  nona22aa1n02x5               g181(.a(new_n276), .b(new_n271), .c(new_n267), .out0(new_n277));
  nanp02aa1n03x5               g182(.a(new_n272), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n268), .b(new_n271), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n279), .b(new_n275), .c(new_n273), .o1(new_n280));
  aoi012aa1n02x5               g185(.a(new_n269), .b(new_n267), .c(new_n270), .o1(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  aoi012aa1n02x7               g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(new_n283));
  inv000aa1n02x5               g188(.a(new_n279), .o1(new_n284));
  aoi013aa1n02x4               g189(.a(new_n284), .b(new_n261), .c(new_n262), .d(new_n264), .o1(new_n285));
  nano22aa1n02x4               g190(.a(new_n285), .b(new_n281), .c(new_n282), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n283), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  norb03aa1n09x5               g194(.a(new_n268), .b(new_n282), .c(new_n271), .out0(new_n290));
  oaih12aa1n02x5               g195(.a(new_n290), .b(new_n275), .c(new_n273), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n289), .b(new_n291), .c(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n290), .o1(new_n294));
  aoi013aa1n02x4               g199(.a(new_n294), .b(new_n261), .c(new_n262), .d(new_n264), .o1(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n289), .c(new_n292), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n293), .b(new_n296), .o1(\s[30] ));
  xnrc02aa1n02x5               g202(.a(\b[30] ), .b(\a[31] ), .out0(new_n298));
  norb02aa1n02x5               g203(.a(new_n290), .b(new_n289), .out0(new_n299));
  oaih12aa1n02x5               g204(.a(new_n299), .b(new_n275), .c(new_n273), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n298), .b(new_n300), .c(new_n301), .o1(new_n302));
  inv000aa1n02x5               g207(.a(new_n299), .o1(new_n303));
  aoi013aa1n02x4               g208(.a(new_n303), .b(new_n261), .c(new_n262), .d(new_n264), .o1(new_n304));
  nano22aa1n02x4               g209(.a(new_n304), .b(new_n298), .c(new_n301), .out0(new_n305));
  norp02aa1n03x5               g210(.a(new_n302), .b(new_n305), .o1(\s[31] ));
  inv000aa1d42x5               g211(.a(\a[3] ), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n215), .b(\b[2] ), .c(new_n307), .out0(\s[3] ));
  xorc02aa1n02x5               g213(.a(\a[4] ), .b(\b[3] ), .out0(new_n309));
  aoib12aa1n02x5               g214(.a(new_n309), .b(new_n307), .c(\b[2] ), .out0(new_n310));
  aoi022aa1n02x5               g215(.a(new_n216), .b(new_n309), .c(new_n104), .d(new_n310), .o1(\s[4] ));
  nanp02aa1n02x5               g216(.a(new_n216), .b(new_n109), .o1(new_n312));
  aoi022aa1n02x5               g217(.a(new_n216), .b(new_n106), .c(new_n108), .d(new_n117), .o1(new_n313));
  norb02aa1n02x5               g218(.a(new_n312), .b(new_n313), .out0(\s[5] ));
  xobna2aa1n03x5               g219(.a(new_n110), .b(new_n312), .c(new_n117), .out0(\s[6] ));
  nona22aa1n02x4               g220(.a(new_n312), .b(new_n110), .c(new_n107), .out0(new_n316));
  aob012aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n03x5               g223(.a(\a[7] ), .b(\b[6] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g225(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


