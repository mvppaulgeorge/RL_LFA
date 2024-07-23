// Benchmark "adder" written by ABC on Wed Jul 17 18:01:32 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n301, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n321, new_n324, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n08x5               g001(.a(\b[2] ), .b(\a[3] ), .o1(new_n97));
  nor002aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nor002aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  tech160nm_finand02aa1n03p5x5 g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nanb02aa1n06x5               g005(.a(new_n97), .b(new_n100), .out0(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  oai112aa1n06x5               g007(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n103));
  nand22aa1n03x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  oai012aa1n09x5               g009(.a(new_n99), .b(new_n104), .c(new_n101), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  norp02aa1n04x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  aoi012aa1n02x5               g012(.a(new_n107), .b(\a[4] ), .c(\b[3] ), .o1(new_n108));
  nand42aa1n04x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nor042aa1n04x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  norb02aa1n03x5               g015(.a(new_n109), .b(new_n110), .out0(new_n111));
  norp02aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  norp02aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  nano32aa1n03x7               g021(.a(new_n116), .b(new_n111), .c(new_n108), .d(new_n106), .out0(new_n117));
  aoi022aa1n12x5               g022(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(new_n118), .o1(new_n119));
  aoi112aa1n03x5               g024(.a(new_n114), .b(new_n112), .c(new_n110), .d(new_n115), .o1(new_n120));
  oai022aa1n06x5               g025(.a(new_n120), .b(new_n119), .c(\b[7] ), .d(\a[8] ), .o1(new_n121));
  tech160nm_fiaoi012aa1n05x5   g026(.a(new_n121), .b(new_n117), .c(new_n105), .o1(new_n122));
  tech160nm_fioaoi03aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  inv000aa1d42x5               g029(.a(\a[10] ), .o1(new_n125));
  inv000aa1d42x5               g030(.a(\b[9] ), .o1(new_n126));
  nand02aa1d16x5               g031(.a(new_n126), .b(new_n125), .o1(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  nanp02aa1n09x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nor002aa1d32x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand42aa1n03x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n06x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  aoai13aa1n06x5               g037(.a(new_n132), .b(new_n128), .c(new_n123), .d(new_n129), .o1(new_n133));
  aoi112aa1n02x5               g038(.a(new_n128), .b(new_n132), .c(new_n123), .d(new_n129), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(\s[11] ));
  inv000aa1d42x5               g040(.a(new_n130), .o1(new_n136));
  nor042aa1n04x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n133), .c(new_n136), .out0(\s[12] ));
  nanp02aa1n02x5               g045(.a(\b[8] ), .b(\a[9] ), .o1(new_n141));
  nano22aa1d15x5               g046(.a(new_n137), .b(new_n129), .c(new_n138), .out0(new_n142));
  oai112aa1n06x5               g047(.a(new_n127), .b(new_n129), .c(\b[8] ), .d(\a[9] ), .o1(new_n143));
  nano32aa1n03x7               g048(.a(new_n143), .b(new_n142), .c(new_n132), .d(new_n141), .out0(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n121), .c(new_n117), .d(new_n105), .o1(new_n145));
  nand02aa1n02x5               g050(.a(new_n130), .b(new_n138), .o1(new_n146));
  inv040aa1n02x5               g051(.a(new_n146), .o1(new_n147));
  aoi113aa1n06x5               g052(.a(new_n147), .b(new_n137), .c(new_n142), .d(new_n143), .e(new_n132), .o1(new_n148));
  nor022aa1n08x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nand42aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(new_n151));
  xobna2aa1n03x5               g056(.a(new_n151), .b(new_n145), .c(new_n148), .out0(\s[13] ));
  inv000aa1d42x5               g057(.a(\a[13] ), .o1(new_n153));
  inv000aa1d42x5               g058(.a(\b[12] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(new_n145), .b(new_n148), .o1(new_n155));
  oaoi03aa1n02x5               g060(.a(new_n153), .b(new_n154), .c(new_n155), .o1(new_n156));
  xnrb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n04x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  and002aa1n12x5               g063(.a(\b[13] ), .b(\a[14] ), .o(new_n159));
  oab012aa1n09x5               g064(.a(new_n159), .b(new_n149), .c(new_n158), .out0(new_n160));
  xnrc02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .out0(new_n161));
  aoi112aa1n03x5               g066(.a(new_n161), .b(new_n151), .c(new_n145), .d(new_n148), .o1(new_n162));
  nor042aa1n03x5               g067(.a(new_n162), .b(new_n160), .o1(new_n163));
  nor042aa1n03x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n163), .b(new_n166), .c(new_n165), .out0(\s[15] ));
  xnrc02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .out0(new_n168));
  oabi12aa1n06x5               g073(.a(new_n168), .b(new_n162), .c(new_n160), .out0(new_n169));
  nor002aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n03x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  tech160nm_fiaoi012aa1n04x5   g077(.a(new_n172), .b(new_n169), .c(new_n165), .o1(new_n173));
  nanp03aa1n03x5               g078(.a(new_n169), .b(new_n165), .c(new_n172), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  nor043aa1n06x5               g080(.a(new_n159), .b(new_n158), .c(new_n149), .o1(new_n176));
  nano22aa1n03x7               g081(.a(new_n170), .b(new_n150), .c(new_n171), .out0(new_n177));
  nanb03aa1n09x5               g082(.a(new_n168), .b(new_n177), .c(new_n176), .out0(new_n178));
  nanp03aa1n03x5               g083(.a(new_n142), .b(new_n143), .c(new_n132), .o1(new_n179));
  nona22aa1n03x5               g084(.a(new_n179), .b(new_n147), .c(new_n137), .out0(new_n180));
  inv040aa1n06x5               g085(.a(new_n178), .o1(new_n181));
  aoi022aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n182));
  oaih12aa1n06x5               g087(.a(new_n182), .b(new_n160), .c(new_n164), .o1(new_n183));
  oai012aa1n12x5               g088(.a(new_n183), .b(\b[15] ), .c(\a[16] ), .o1(new_n184));
  aoi012aa1d18x5               g089(.a(new_n184), .b(new_n180), .c(new_n181), .o1(new_n185));
  oai012aa1n12x5               g090(.a(new_n185), .b(new_n145), .c(new_n178), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n02x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  oabi12aa1n03x5               g093(.a(new_n184), .b(new_n148), .c(new_n178), .out0(new_n189));
  nano22aa1n03x7               g094(.a(new_n122), .b(new_n181), .c(new_n144), .out0(new_n190));
  norp03aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(new_n188), .o1(new_n191));
  aoi012aa1n02x5               g096(.a(new_n191), .b(\a[17] ), .c(\b[16] ), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\a[18] ), .o1(new_n195));
  xroi22aa1d04x5               g100(.a(new_n194), .b(\b[16] ), .c(new_n195), .d(\b[17] ), .out0(new_n196));
  inv000aa1d42x5               g101(.a(\b[17] ), .o1(new_n197));
  oao003aa1n03x5               g102(.a(new_n195), .b(new_n197), .c(new_n188), .carry(new_n198));
  nor022aa1n08x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand42aa1n06x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n198), .c(new_n186), .d(new_n196), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n201), .b(new_n198), .c(new_n186), .d(new_n196), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n06x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand02aa1n03x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  nona22aa1n02x5               g113(.a(new_n202), .b(new_n208), .c(new_n199), .out0(new_n209));
  orn002aa1n24x5               g114(.a(\a[19] ), .b(\b[18] ), .o(new_n210));
  aobi12aa1n02x7               g115(.a(new_n208), .b(new_n202), .c(new_n210), .out0(new_n211));
  norb02aa1n03x4               g116(.a(new_n209), .b(new_n211), .out0(\s[20] ));
  aoi113aa1n02x5               g117(.a(new_n98), .b(new_n97), .c(new_n103), .d(new_n102), .e(new_n100), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(\b[3] ), .b(\a[4] ), .o1(new_n214));
  nano22aa1n02x4               g119(.a(new_n107), .b(new_n106), .c(new_n214), .out0(new_n215));
  nano23aa1n02x4               g120(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n216));
  nand03aa1n02x5               g121(.a(new_n216), .b(new_n215), .c(new_n111), .o1(new_n217));
  oab012aa1n02x4               g122(.a(new_n107), .b(new_n120), .c(new_n119), .out0(new_n218));
  oai012aa1n06x5               g123(.a(new_n218), .b(new_n217), .c(new_n213), .o1(new_n219));
  nanp03aa1n02x5               g124(.a(new_n132), .b(new_n139), .c(new_n129), .o1(new_n220));
  nanb02aa1n02x5               g125(.a(new_n143), .b(new_n141), .out0(new_n221));
  nona32aa1n09x5               g126(.a(new_n219), .b(new_n178), .c(new_n221), .d(new_n220), .out0(new_n222));
  nano23aa1n06x5               g127(.a(new_n199), .b(new_n206), .c(new_n207), .d(new_n200), .out0(new_n223));
  nanp02aa1n02x5               g128(.a(new_n196), .b(new_n223), .o1(new_n224));
  oaoi03aa1n12x5               g129(.a(new_n195), .b(new_n197), .c(new_n188), .o1(new_n225));
  nona23aa1n09x5               g130(.a(new_n207), .b(new_n200), .c(new_n199), .d(new_n206), .out0(new_n226));
  oaoi03aa1n09x5               g131(.a(\a[20] ), .b(\b[19] ), .c(new_n210), .o1(new_n227));
  oabi12aa1n18x5               g132(.a(new_n227), .b(new_n226), .c(new_n225), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n224), .c(new_n222), .d(new_n185), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[22] ), .b(\b[21] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(\s[22] ));
  inv000aa1d42x5               g142(.a(\a[21] ), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\a[22] ), .o1(new_n239));
  xroi22aa1d06x4               g144(.a(new_n238), .b(\b[20] ), .c(new_n239), .d(\b[21] ), .out0(new_n240));
  nand23aa1n03x5               g145(.a(new_n240), .b(new_n196), .c(new_n223), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\b[21] ), .o1(new_n242));
  oao003aa1n02x5               g147(.a(new_n239), .b(new_n242), .c(new_n232), .carry(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(new_n228), .c(new_n240), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n241), .c(new_n222), .d(new_n185), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[23] ), .b(\b[22] ), .out0(new_n248));
  norp02aa1n02x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  nand42aa1n03x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  norb02aa1n03x4               g155(.a(new_n250), .b(new_n249), .out0(new_n251));
  aoi112aa1n02x5               g156(.a(new_n247), .b(new_n251), .c(new_n245), .d(new_n248), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n251), .b(new_n247), .c(new_n245), .d(new_n248), .o1(new_n253));
  norb02aa1n03x4               g158(.a(new_n253), .b(new_n252), .out0(\s[24] ));
  and002aa1n06x5               g159(.a(new_n248), .b(new_n251), .o(new_n255));
  inv000aa1n02x5               g160(.a(new_n255), .o1(new_n256));
  nano32aa1n02x4               g161(.a(new_n256), .b(new_n240), .c(new_n196), .d(new_n223), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n240), .b(new_n227), .c(new_n223), .d(new_n198), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n243), .o1(new_n259));
  oai012aa1n02x5               g164(.a(new_n250), .b(new_n249), .c(new_n247), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n256), .c(new_n258), .d(new_n259), .o1(new_n261));
  tech160nm_fixorc02aa1n04x5   g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n261), .c(new_n186), .d(new_n257), .o1(new_n263));
  aoi112aa1n02x5               g168(.a(new_n262), .b(new_n261), .c(new_n186), .d(new_n257), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n263), .b(new_n264), .out0(\s[25] ));
  nor042aa1n03x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  nona22aa1n02x5               g172(.a(new_n263), .b(new_n267), .c(new_n266), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n266), .o1(new_n269));
  aobi12aa1n03x5               g174(.a(new_n267), .b(new_n263), .c(new_n269), .out0(new_n270));
  norb02aa1n03x4               g175(.a(new_n268), .b(new_n270), .out0(\s[26] ));
  nor042aa1n04x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n273), .b(new_n272), .out0(new_n274));
  and002aa1n06x5               g179(.a(new_n267), .b(new_n262), .o(new_n275));
  nano22aa1d15x5               g180(.a(new_n241), .b(new_n255), .c(new_n275), .out0(new_n276));
  oai012aa1n06x5               g181(.a(new_n276), .b(new_n190), .c(new_n189), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n278));
  aobi12aa1n06x5               g183(.a(new_n278), .b(new_n261), .c(new_n275), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n274), .b(new_n279), .c(new_n277), .out0(\s[27] ));
  xorc02aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n255), .b(new_n243), .c(new_n228), .d(new_n240), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n275), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n278), .b(new_n283), .c(new_n282), .d(new_n260), .o1(new_n284));
  aoi112aa1n02x7               g189(.a(new_n284), .b(new_n272), .c(new_n276), .d(new_n186), .o1(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n273), .c(new_n281), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n272), .o1(new_n287));
  nand03aa1n02x5               g192(.a(new_n279), .b(new_n277), .c(new_n287), .o1(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n281), .b(new_n288), .c(new_n273), .o1(new_n289));
  nor002aa1n02x5               g194(.a(new_n289), .b(new_n286), .o1(\s[28] ));
  inv020aa1n02x5               g195(.a(new_n276), .o1(new_n291));
  aoi012aa1n06x5               g196(.a(new_n291), .b(new_n222), .c(new_n185), .o1(new_n292));
  and002aa1n02x5               g197(.a(new_n281), .b(new_n274), .o(new_n293));
  oai012aa1n03x5               g198(.a(new_n293), .b(new_n284), .c(new_n292), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  aoi012aa1n03x5               g201(.a(new_n296), .b(new_n294), .c(new_n295), .o1(new_n297));
  aobi12aa1n03x5               g202(.a(new_n293), .b(new_n279), .c(new_n277), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n295), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[29] ));
  nanp02aa1n02x5               g205(.a(\b[0] ), .b(\a[1] ), .o1(new_n301));
  xorb03aa1n02x5               g206(.a(new_n301), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g207(.a(new_n296), .b(new_n281), .c(new_n274), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n303), .b(new_n284), .c(new_n292), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  aoi012aa1n03x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n03x5               g212(.a(new_n303), .b(new_n279), .c(new_n277), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  nano23aa1n02x4               g215(.a(new_n306), .b(new_n296), .c(new_n281), .d(new_n274), .out0(new_n311));
  oaih12aa1n02x5               g216(.a(new_n311), .b(new_n284), .c(new_n292), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  aoi012aa1n03x5               g219(.a(new_n314), .b(new_n312), .c(new_n313), .o1(new_n315));
  aobi12aa1n03x5               g220(.a(new_n311), .b(new_n279), .c(new_n277), .out0(new_n316));
  nano22aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n314), .out0(new_n317));
  norp02aa1n03x5               g222(.a(new_n315), .b(new_n317), .o1(\s[31] ));
  xnbna2aa1n03x5               g223(.a(new_n101), .b(new_n103), .c(new_n102), .out0(\s[3] ));
  norb02aa1n02x5               g224(.a(new_n214), .b(new_n98), .out0(new_n320));
  aoi113aa1n02x5               g225(.a(new_n97), .b(new_n320), .c(new_n103), .d(new_n102), .e(new_n100), .o1(new_n321));
  aoi012aa1n02x5               g226(.a(new_n321), .b(new_n105), .c(new_n320), .o1(\s[4] ));
  xobna2aa1n03x5               g227(.a(new_n111), .b(new_n105), .c(new_n214), .out0(\s[5] ));
  aoai13aa1n02x5               g228(.a(new_n109), .b(new_n110), .c(new_n105), .d(new_n214), .o1(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g230(.a(\a[6] ), .b(\b[5] ), .c(new_n324), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g232(.a(new_n112), .b(new_n326), .c(new_n113), .o1(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g234(.a(new_n219), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


