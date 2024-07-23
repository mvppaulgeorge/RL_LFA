// Benchmark "adder" written by ABC on Thu Jul 18 11:16:25 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n317, new_n319, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor002aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n03x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand22aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n06x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  nor022aa1n08x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand22aa1n09x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor022aa1n16x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n09x5               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  ao0012aa1n03x7               g011(.a(new_n102), .b(new_n104), .c(new_n103), .o(new_n107));
  oabi12aa1n18x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .out0(new_n108));
  nand42aa1d28x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor042aa1n04x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  norb02aa1n03x4               g015(.a(new_n109), .b(new_n110), .out0(new_n111));
  nor042aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1n04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norb02aa1n06x5               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[6] ), .b(\a[7] ), .out0(new_n115));
  tech160nm_fixnrc02aa1n04x5   g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nano23aa1n06x5               g021(.a(new_n116), .b(new_n115), .c(new_n114), .d(new_n111), .out0(new_n117));
  aoi112aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(new_n109), .o1(new_n119));
  xorc02aa1n06x5               g024(.a(\a[7] ), .b(\b[6] ), .out0(new_n120));
  oab012aa1n03x5               g025(.a(new_n110), .b(\a[5] ), .c(\b[4] ), .out0(new_n121));
  nona23aa1n03x5               g026(.a(new_n120), .b(new_n114), .c(new_n121), .d(new_n119), .out0(new_n122));
  nona22aa1n03x5               g027(.a(new_n122), .b(new_n118), .c(new_n112), .out0(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n108), .d(new_n117), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n97), .out0(\s[10] ));
  nand02aa1n06x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor002aa1d24x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand42aa1d28x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanb02aa1n06x5               g035(.a(new_n129), .b(new_n130), .out0(new_n131));
  oai112aa1n03x5               g036(.a(new_n125), .b(new_n97), .c(\b[9] ), .d(\a[10] ), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n131), .b(new_n132), .c(new_n128), .out0(\s[11] ));
  inv000aa1d42x5               g038(.a(new_n129), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n128), .o1(new_n135));
  nona22aa1n02x4               g040(.a(new_n132), .b(new_n131), .c(new_n135), .out0(new_n136));
  nor002aa1n06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1n16x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n06x4               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n136), .c(new_n134), .out0(\s[12] ));
  nand22aa1n03x5               g045(.a(new_n117), .b(new_n108), .o1(new_n141));
  nanb02aa1n03x5               g046(.a(new_n112), .b(new_n113), .out0(new_n142));
  oai022aa1n02x5               g047(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n143));
  nano23aa1n02x4               g048(.a(new_n115), .b(new_n142), .c(new_n143), .d(new_n109), .out0(new_n144));
  nor003aa1n02x5               g049(.a(new_n144), .b(new_n118), .c(new_n112), .o1(new_n145));
  nano23aa1n06x5               g050(.a(new_n129), .b(new_n137), .c(new_n138), .d(new_n130), .out0(new_n146));
  nand23aa1d12x5               g051(.a(new_n146), .b(new_n124), .c(new_n126), .o1(new_n147));
  aoi112aa1n02x5               g052(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n148));
  nanb02aa1n02x5               g053(.a(new_n137), .b(new_n138), .out0(new_n149));
  oai022aa1n02x5               g054(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n150));
  nano23aa1n03x7               g055(.a(new_n149), .b(new_n131), .c(new_n150), .d(new_n128), .out0(new_n151));
  nor043aa1n03x5               g056(.a(new_n151), .b(new_n148), .c(new_n137), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n147), .c(new_n141), .d(new_n145), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1d28x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  tech160nm_fiaoi012aa1n05x5   g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g063(.a(new_n147), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n123), .c(new_n108), .d(new_n117), .o1(new_n160));
  nor042aa1n06x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand02aa1d24x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nano23aa1d15x5               g067(.a(new_n155), .b(new_n161), .c(new_n162), .d(new_n156), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  tech160nm_fioai012aa1n05x5   g069(.a(new_n162), .b(new_n161), .c(new_n155), .o1(new_n165));
  aoai13aa1n04x5               g070(.a(new_n165), .b(new_n164), .c(new_n160), .d(new_n152), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n08x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand22aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n06x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  nor042aa1n03x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nand02aa1n04x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n12x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoi112aa1n02x5               g078(.a(new_n173), .b(new_n168), .c(new_n166), .d(new_n170), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n173), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(\s[16] ));
  nand23aa1d12x5               g081(.a(new_n163), .b(new_n170), .c(new_n173), .o1(new_n177));
  nor042aa1n09x5               g082(.a(new_n147), .b(new_n177), .o1(new_n178));
  aoai13aa1n12x5               g083(.a(new_n178), .b(new_n123), .c(new_n108), .d(new_n117), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n130), .b(new_n129), .out0(new_n180));
  nor002aa1n02x5               g085(.a(\b[8] ), .b(\a[9] ), .o1(new_n181));
  oab012aa1n02x4               g086(.a(new_n181), .b(\a[10] ), .c(\b[9] ), .out0(new_n182));
  nona23aa1n02x4               g087(.a(new_n139), .b(new_n180), .c(new_n182), .d(new_n135), .out0(new_n183));
  nona22aa1n02x4               g088(.a(new_n183), .b(new_n148), .c(new_n137), .out0(new_n184));
  inv030aa1n02x5               g089(.a(new_n177), .o1(new_n185));
  nona23aa1n09x5               g090(.a(new_n172), .b(new_n169), .c(new_n168), .d(new_n171), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n168), .b(new_n172), .o1(new_n187));
  oai122aa1n06x5               g092(.a(new_n187), .b(new_n186), .c(new_n165), .d(\b[15] ), .e(\a[16] ), .o1(new_n188));
  aoi012aa1n12x5               g093(.a(new_n188), .b(new_n184), .c(new_n185), .o1(new_n189));
  xorc02aa1n02x5               g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n190), .b(new_n179), .c(new_n189), .out0(\s[17] ));
  inv000aa1d42x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\b[16] ), .o1(new_n193));
  nand42aa1n03x5               g098(.a(new_n193), .b(new_n192), .o1(new_n194));
  nand22aa1n03x5               g099(.a(new_n141), .b(new_n145), .o1(new_n195));
  oabi12aa1n03x5               g100(.a(new_n188), .b(new_n152), .c(new_n177), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n190), .b(new_n196), .c(new_n195), .d(new_n178), .o1(new_n197));
  nor042aa1n09x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nand42aa1n04x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n197), .c(new_n194), .out0(\s[18] ));
  nand42aa1n03x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  nano32aa1d12x5               g107(.a(new_n198), .b(new_n194), .c(new_n199), .d(new_n202), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n199), .b(new_n198), .c(new_n192), .d(new_n193), .o1(new_n205));
  aoai13aa1n04x5               g110(.a(new_n205), .b(new_n204), .c(new_n179), .d(new_n189), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand02aa1n04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nor022aa1n12x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand02aa1n06x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  aoi112aa1n02x7               g118(.a(new_n209), .b(new_n213), .c(new_n206), .d(new_n210), .o1(new_n214));
  aoai13aa1n03x5               g119(.a(new_n213), .b(new_n209), .c(new_n206), .d(new_n210), .o1(new_n215));
  norb02aa1n02x7               g120(.a(new_n215), .b(new_n214), .out0(\s[20] ));
  nano23aa1n06x5               g121(.a(new_n209), .b(new_n211), .c(new_n212), .d(new_n210), .out0(new_n217));
  nanp03aa1n02x5               g122(.a(new_n217), .b(new_n190), .c(new_n200), .o1(new_n218));
  nona23aa1n09x5               g123(.a(new_n212), .b(new_n210), .c(new_n209), .d(new_n211), .out0(new_n219));
  aoi012aa1d18x5               g124(.a(new_n211), .b(new_n209), .c(new_n212), .o1(new_n220));
  oai012aa1n12x5               g125(.a(new_n220), .b(new_n219), .c(new_n205), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n04x5               g127(.a(new_n222), .b(new_n218), .c(new_n179), .d(new_n189), .o1(new_n223));
  xorb03aa1n02x5               g128(.a(new_n223), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .out0(new_n226));
  xorc02aa1n02x5               g131(.a(\a[22] ), .b(\b[21] ), .out0(new_n227));
  aoi112aa1n02x7               g132(.a(new_n225), .b(new_n227), .c(new_n223), .d(new_n226), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n227), .b(new_n225), .c(new_n223), .d(new_n226), .o1(new_n229));
  norb02aa1n02x7               g134(.a(new_n229), .b(new_n228), .out0(\s[22] ));
  inv000aa1d42x5               g135(.a(\a[21] ), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\a[22] ), .o1(new_n232));
  xroi22aa1d04x5               g137(.a(new_n231), .b(\b[20] ), .c(new_n232), .d(\b[21] ), .out0(new_n233));
  nanp03aa1n03x5               g138(.a(new_n233), .b(new_n203), .c(new_n217), .o1(new_n234));
  inv040aa1n03x5               g139(.a(new_n205), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n220), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n233), .b(new_n236), .c(new_n217), .d(new_n235), .o1(new_n237));
  inv000aa1d42x5               g142(.a(\b[21] ), .o1(new_n238));
  oaoi03aa1n12x5               g143(.a(new_n232), .b(new_n238), .c(new_n225), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(new_n237), .b(new_n239), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n04x5               g146(.a(new_n241), .b(new_n234), .c(new_n179), .d(new_n189), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  tech160nm_fixorc02aa1n02p5x5 g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  xorc02aa1n02x5               g150(.a(\a[24] ), .b(\b[23] ), .out0(new_n246));
  aoi112aa1n02x5               g151(.a(new_n244), .b(new_n246), .c(new_n242), .d(new_n245), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n245), .o1(new_n248));
  norb02aa1n02x7               g153(.a(new_n248), .b(new_n247), .out0(\s[24] ));
  and002aa1n06x5               g154(.a(new_n246), .b(new_n245), .o(new_n250));
  nanb03aa1n02x5               g155(.a(new_n218), .b(new_n250), .c(new_n233), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n250), .o1(new_n252));
  orn002aa1n02x5               g157(.a(\a[23] ), .b(\b[22] ), .o(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .carry(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n252), .c(new_n237), .d(new_n239), .o1(new_n255));
  inv040aa1n03x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n251), .c(new_n179), .d(new_n189), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  tech160nm_fixorc02aa1n05x5   g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  tech160nm_fixorc02aa1n05x5   g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  aoi112aa1n02x7               g166(.a(new_n259), .b(new_n261), .c(new_n257), .d(new_n260), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n263));
  norb02aa1n02x7               g168(.a(new_n263), .b(new_n262), .out0(\s[26] ));
  and002aa1n09x5               g169(.a(new_n261), .b(new_n260), .o(new_n265));
  nano22aa1n03x7               g170(.a(new_n234), .b(new_n250), .c(new_n265), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n196), .c(new_n195), .d(new_n178), .o1(new_n267));
  orn002aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .o(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n269));
  aobi12aa1n06x5               g174(.a(new_n269), .b(new_n255), .c(new_n265), .out0(new_n270));
  xorc02aa1n12x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  inv040aa1n03x5               g178(.a(new_n273), .o1(new_n274));
  aobi12aa1n02x7               g179(.a(new_n271), .b(new_n270), .c(new_n267), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  nano22aa1n03x5               g181(.a(new_n275), .b(new_n274), .c(new_n276), .out0(new_n277));
  inv000aa1n02x5               g182(.a(new_n266), .o1(new_n278));
  aoi012aa1n06x5               g183(.a(new_n278), .b(new_n179), .c(new_n189), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n239), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n250), .b(new_n280), .c(new_n221), .d(new_n233), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n265), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n269), .b(new_n282), .c(new_n281), .d(new_n254), .o1(new_n283));
  oaih12aa1n02x5               g188(.a(new_n271), .b(new_n283), .c(new_n279), .o1(new_n284));
  aoi012aa1n03x5               g189(.a(new_n276), .b(new_n284), .c(new_n274), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n285), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n271), .b(new_n276), .out0(new_n287));
  aobi12aa1n03x5               g192(.a(new_n287), .b(new_n270), .c(new_n267), .out0(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  nano22aa1n03x5               g195(.a(new_n288), .b(new_n289), .c(new_n290), .out0(new_n291));
  oaih12aa1n02x5               g196(.a(new_n287), .b(new_n283), .c(new_n279), .o1(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n290), .b(new_n292), .c(new_n289), .o1(new_n293));
  norp02aa1n03x5               g198(.a(new_n293), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n271), .b(new_n290), .c(new_n276), .out0(new_n296));
  aobi12aa1n03x5               g201(.a(new_n296), .b(new_n270), .c(new_n267), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  oaih12aa1n02x5               g205(.a(new_n296), .b(new_n283), .c(new_n279), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n299), .b(new_n301), .c(new_n298), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n300), .o1(\s[30] ));
  norb02aa1n02x5               g208(.a(new_n296), .b(new_n299), .out0(new_n304));
  aobi12aa1n03x5               g209(.a(new_n304), .b(new_n270), .c(new_n267), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n304), .b(new_n283), .c(new_n279), .o1(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g220(.a(new_n116), .b(new_n108), .out0(new_n316));
  tech160nm_fioai012aa1n03p5x5 g221(.a(new_n316), .b(\b[4] ), .c(\a[5] ), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g223(.a(new_n120), .b(new_n110), .c(new_n317), .d(new_n109), .o1(new_n319));
  aoi112aa1n02x5               g224(.a(new_n120), .b(new_n110), .c(new_n317), .d(new_n109), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n319), .b(new_n320), .out0(\s[7] ));
  orn002aa1n02x5               g226(.a(\a[7] ), .b(\b[6] ), .o(new_n322));
  xnbna2aa1n03x5               g227(.a(new_n114), .b(new_n319), .c(new_n322), .out0(\s[8] ));
  xnbna2aa1n03x5               g228(.a(new_n124), .b(new_n141), .c(new_n145), .out0(\s[9] ));
endmodule


