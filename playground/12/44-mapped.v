// Benchmark "adder" written by ABC on Wed Jul 17 18:30:28 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n291, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n309,
    new_n311, new_n312, new_n315, new_n317, new_n318, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nor022aa1n08x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand02aa1n08x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor022aa1n08x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand02aa1n08x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1n09x5               g008(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n104));
  nor042aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n24x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n06x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nor042aa1n04x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  aoi022aa1d24x5               g013(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n109));
  oai012aa1n12x5               g014(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n110));
  oab012aa1n04x5               g015(.a(new_n105), .b(\a[4] ), .c(\b[3] ), .out0(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  xorc02aa1n12x5               g017(.a(\a[5] ), .b(\b[4] ), .out0(new_n113));
  nor042aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand42aa1n06x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  norb02aa1n06x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nand23aa1n04x5               g021(.a(new_n113), .b(new_n112), .c(new_n116), .o1(new_n117));
  aoi112aa1n09x5               g022(.a(new_n117), .b(new_n104), .c(new_n110), .d(new_n111), .o1(new_n118));
  aoi112aa1n09x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  nano23aa1n09x5               g024(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n120));
  orn002aa1n03x5               g025(.a(\a[5] ), .b(\b[4] ), .o(new_n121));
  oaoi03aa1n09x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .o1(new_n122));
  nand42aa1n08x5               g027(.a(new_n120), .b(new_n122), .o1(new_n123));
  nona22aa1d18x5               g028(.a(new_n123), .b(new_n119), .c(new_n100), .out0(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  oai012aa1n02x5               g030(.a(new_n125), .b(new_n118), .c(new_n124), .o1(new_n126));
  xnrc02aa1n12x5               g031(.a(\b[9] ), .b(\a[10] ), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n126), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(new_n99), .o1(new_n130));
  nand42aa1n04x5               g035(.a(new_n110), .b(new_n111), .o1(new_n131));
  nanb03aa1n06x5               g036(.a(new_n117), .b(new_n120), .c(new_n131), .out0(new_n132));
  aoi112aa1n03x5               g037(.a(new_n119), .b(new_n100), .c(new_n120), .d(new_n122), .o1(new_n133));
  nand02aa1n03x5               g038(.a(new_n132), .b(new_n133), .o1(new_n134));
  aoai13aa1n06x5               g039(.a(new_n128), .b(new_n130), .c(new_n134), .d(new_n125), .o1(new_n135));
  oaoi03aa1n12x5               g040(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  tech160nm_fixnrc02aa1n04x5   g042(.a(\b[10] ), .b(\a[11] ), .out0(new_n138));
  xobna2aa1n03x5               g043(.a(new_n138), .b(new_n135), .c(new_n137), .out0(\s[11] ));
  nand22aa1n04x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nona22aa1n03x5               g045(.a(new_n135), .b(new_n136), .c(new_n138), .out0(new_n141));
  xnrc02aa1n12x5               g046(.a(\b[11] ), .b(\a[12] ), .out0(new_n142));
  nanp03aa1n02x5               g047(.a(new_n141), .b(new_n140), .c(new_n142), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n142), .o1(new_n144));
  aob012aa1n02x5               g049(.a(new_n144), .b(new_n141), .c(new_n140), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n143), .o1(\s[12] ));
  nona23aa1d18x5               g051(.a(new_n125), .b(new_n144), .c(new_n138), .d(new_n127), .out0(new_n147));
  oabi12aa1n06x5               g052(.a(new_n147), .b(new_n118), .c(new_n124), .out0(new_n148));
  nand42aa1n02x5               g053(.a(\b[9] ), .b(\a[10] ), .o1(new_n149));
  oaih22aa1d12x5               g054(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n150));
  nand23aa1n06x5               g055(.a(new_n150), .b(new_n149), .c(new_n140), .o1(new_n151));
  oa0022aa1n06x5               g056(.a(\a[12] ), .b(\b[11] ), .c(\a[11] ), .d(\b[10] ), .o(new_n152));
  aoi022aa1d24x5               g057(.a(new_n151), .b(new_n152), .c(\b[11] ), .d(\a[12] ), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  norp02aa1n24x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1n04x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n148), .c(new_n154), .out0(\s[13] ));
  aoai13aa1n02x5               g063(.a(new_n154), .b(new_n147), .c(new_n132), .d(new_n133), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n155), .b(new_n159), .c(new_n156), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n12x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n04x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n09x5               g068(.a(new_n163), .b(new_n156), .c(new_n155), .d(new_n162), .out0(new_n164));
  oai012aa1n02x7               g069(.a(new_n163), .b(new_n162), .c(new_n155), .o1(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n164), .c(new_n148), .d(new_n154), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n16x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanb02aa1d24x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  nor022aa1n08x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand22aa1n09x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanb02aa1d36x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  aoi112aa1n03x5               g080(.a(new_n168), .b(new_n175), .c(new_n166), .d(new_n171), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n175), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n177));
  norb02aa1n03x4               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  nano23aa1n09x5               g083(.a(new_n155), .b(new_n162), .c(new_n163), .d(new_n156), .out0(new_n179));
  nona22aa1n09x5               g084(.a(new_n179), .b(new_n170), .c(new_n174), .out0(new_n180));
  nor042aa1n12x5               g085(.a(new_n147), .b(new_n180), .o1(new_n181));
  oai012aa1d24x5               g086(.a(new_n181), .b(new_n118), .c(new_n124), .o1(new_n182));
  nor003aa1n03x5               g087(.a(new_n164), .b(new_n170), .c(new_n174), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n173), .b(new_n169), .o1(new_n184));
  oaoi13aa1n06x5               g089(.a(new_n184), .b(new_n165), .c(\a[15] ), .d(\b[14] ), .o1(new_n185));
  aoi112aa1n09x5               g090(.a(new_n185), .b(new_n172), .c(new_n183), .d(new_n153), .o1(new_n186));
  nand02aa1d08x5               g091(.a(new_n182), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g093(.a(\a[18] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d06x4               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  nanp02aa1n02x5               g099(.a(new_n191), .b(new_n190), .o1(new_n195));
  oaoi03aa1n12x5               g100(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n196));
  nor002aa1n16x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanp02aa1n04x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n196), .c(new_n187), .d(new_n194), .o1(new_n200));
  aoi112aa1n02x5               g105(.a(new_n199), .b(new_n196), .c(new_n187), .d(new_n194), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n24x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand02aa1n06x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  nona22aa1n02x5               g111(.a(new_n200), .b(new_n206), .c(new_n197), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n206), .o1(new_n208));
  oaoi13aa1n06x5               g113(.a(new_n208), .b(new_n200), .c(\a[19] ), .d(\b[18] ), .o1(new_n209));
  norb02aa1n03x4               g114(.a(new_n207), .b(new_n209), .out0(\s[20] ));
  nano23aa1n09x5               g115(.a(new_n197), .b(new_n204), .c(new_n205), .d(new_n198), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n194), .b(new_n211), .o1(new_n212));
  oai022aa1n04x5               g117(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n213));
  oaib12aa1n09x5               g118(.a(new_n213), .b(new_n189), .c(\b[17] ), .out0(new_n214));
  nona23aa1n09x5               g119(.a(new_n205), .b(new_n198), .c(new_n197), .d(new_n204), .out0(new_n215));
  aoi012aa1n12x5               g120(.a(new_n204), .b(new_n197), .c(new_n205), .o1(new_n216));
  oai012aa1d24x5               g121(.a(new_n216), .b(new_n215), .c(new_n214), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n212), .c(new_n182), .d(new_n186), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n02x7               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n225), .b(new_n224), .out0(\s[22] ));
  inv000aa1d42x5               g131(.a(\a[21] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\a[22] ), .o1(new_n228));
  xroi22aa1d06x4               g133(.a(new_n227), .b(\b[20] ), .c(new_n228), .d(\b[21] ), .out0(new_n229));
  nand03aa1n02x5               g134(.a(new_n229), .b(new_n194), .c(new_n211), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oaoi03aa1n12x5               g136(.a(new_n228), .b(new_n231), .c(new_n221), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n217), .c(new_n229), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n230), .c(new_n182), .d(new_n186), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xorc02aa1n12x5               g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  aoi112aa1n03x5               g144(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n241));
  norb02aa1n03x4               g146(.a(new_n241), .b(new_n240), .out0(\s[24] ));
  and002aa1n06x5               g147(.a(new_n239), .b(new_n238), .o(new_n243));
  inv000aa1n02x5               g148(.a(new_n243), .o1(new_n244));
  nano32aa1n06x5               g149(.a(new_n244), .b(new_n229), .c(new_n194), .d(new_n211), .out0(new_n245));
  inv020aa1n03x5               g150(.a(new_n216), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n229), .b(new_n246), .c(new_n211), .d(new_n196), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n248));
  oab012aa1n02x4               g153(.a(new_n248), .b(\a[24] ), .c(\b[23] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n244), .c(new_n247), .d(new_n232), .o1(new_n250));
  aoi012aa1n03x5               g155(.a(new_n250), .b(new_n187), .c(new_n245), .o1(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[24] ), .b(\a[25] ), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  xnrc02aa1n02x5               g158(.a(new_n251), .b(new_n253), .out0(\s[25] ));
  nor042aa1n03x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n253), .b(new_n250), .c(new_n187), .d(new_n245), .o1(new_n257));
  xnrc02aa1n06x5               g162(.a(\b[25] ), .b(\a[26] ), .out0(new_n258));
  nand43aa1n02x5               g163(.a(new_n257), .b(new_n256), .c(new_n258), .o1(new_n259));
  tech160nm_fiaoi012aa1n05x5   g164(.a(new_n258), .b(new_n257), .c(new_n256), .o1(new_n260));
  norb02aa1n03x4               g165(.a(new_n259), .b(new_n260), .out0(\s[26] ));
  nanp02aa1n02x5               g166(.a(new_n183), .b(new_n153), .o1(new_n262));
  nona22aa1n02x4               g167(.a(new_n262), .b(new_n185), .c(new_n172), .out0(new_n263));
  nor042aa1n04x5               g168(.a(new_n258), .b(new_n252), .o1(new_n264));
  nano22aa1n03x7               g169(.a(new_n230), .b(new_n243), .c(new_n264), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n263), .c(new_n134), .d(new_n181), .o1(new_n266));
  oao003aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n267));
  aobi12aa1n06x5               g172(.a(new_n267), .b(new_n250), .c(new_n264), .out0(new_n268));
  xorc02aa1n12x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n268), .c(new_n266), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  inv040aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  aobi12aa1n02x7               g177(.a(new_n269), .b(new_n268), .c(new_n266), .out0(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  nano22aa1n03x5               g179(.a(new_n273), .b(new_n272), .c(new_n274), .out0(new_n275));
  aobi12aa1n06x5               g180(.a(new_n265), .b(new_n182), .c(new_n186), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n243), .b(new_n233), .c(new_n217), .d(new_n229), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n264), .o1(new_n278));
  aoai13aa1n04x5               g183(.a(new_n267), .b(new_n278), .c(new_n277), .d(new_n249), .o1(new_n279));
  oaih12aa1n02x5               g184(.a(new_n269), .b(new_n279), .c(new_n276), .o1(new_n280));
  aoi012aa1n03x5               g185(.a(new_n274), .b(new_n280), .c(new_n272), .o1(new_n281));
  nor002aa1n02x5               g186(.a(new_n281), .b(new_n275), .o1(\s[28] ));
  xnrc02aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .out0(new_n283));
  norb02aa1n02x5               g188(.a(new_n269), .b(new_n274), .out0(new_n284));
  oaih12aa1n02x5               g189(.a(new_n284), .b(new_n279), .c(new_n276), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n286));
  aoi012aa1n03x5               g191(.a(new_n283), .b(new_n285), .c(new_n286), .o1(new_n287));
  aobi12aa1n02x7               g192(.a(new_n284), .b(new_n268), .c(new_n266), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n283), .c(new_n286), .out0(new_n289));
  nor002aa1n02x5               g194(.a(new_n287), .b(new_n289), .o1(\s[29] ));
  nanp02aa1n02x5               g195(.a(\b[0] ), .b(\a[1] ), .o1(new_n291));
  xorb03aa1n02x5               g196(.a(new_n291), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  norb03aa1n02x5               g198(.a(new_n269), .b(new_n283), .c(new_n274), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n279), .c(new_n276), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n296));
  aoi012aa1n03x5               g201(.a(new_n293), .b(new_n295), .c(new_n296), .o1(new_n297));
  aobi12aa1n03x5               g202(.a(new_n294), .b(new_n268), .c(new_n266), .out0(new_n298));
  nano22aa1n02x4               g203(.a(new_n298), .b(new_n293), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[30] ));
  norb02aa1n02x5               g205(.a(new_n294), .b(new_n293), .out0(new_n301));
  aobi12aa1n02x7               g206(.a(new_n301), .b(new_n268), .c(new_n266), .out0(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  oaih12aa1n02x5               g210(.a(new_n301), .b(new_n279), .c(new_n276), .o1(new_n306));
  aoi012aa1n03x5               g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  nor002aa1n02x5               g212(.a(new_n307), .b(new_n305), .o1(\s[31] ));
  norp03aa1n02x5               g213(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n309));
  norb02aa1n02x5               g214(.a(new_n110), .b(new_n309), .out0(\s[3] ));
  xorc02aa1n02x5               g215(.a(\a[4] ), .b(\b[3] ), .out0(new_n311));
  oaoi13aa1n02x5               g216(.a(new_n105), .b(new_n106), .c(new_n109), .d(new_n108), .o1(new_n312));
  mtn022aa1n02x5               g217(.a(new_n131), .b(new_n312), .sa(new_n311), .o1(\s[4] ));
  xobna2aa1n03x5               g218(.a(new_n113), .b(new_n131), .c(new_n112), .out0(\s[5] ));
  nanp03aa1n02x5               g219(.a(new_n131), .b(new_n112), .c(new_n113), .o1(new_n315));
  xnbna2aa1n03x5               g220(.a(new_n116), .b(new_n315), .c(new_n121), .out0(\s[6] ));
  aoi012aa1n02x5               g221(.a(new_n117), .b(new_n110), .c(new_n111), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n122), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi13aa1n02x5               g224(.a(new_n102), .b(new_n103), .c(new_n317), .d(new_n122), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g226(.a(new_n125), .b(new_n132), .c(new_n133), .out0(\s[9] ));
endmodule


