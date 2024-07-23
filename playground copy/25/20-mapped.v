// Benchmark "adder" written by ABC on Thu Jul 11 12:44:43 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n315, new_n316, new_n318, new_n319, new_n321;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  160nm_fiao0012aa1n02p5x5     g011(.a(new_n102), .b(new_n104), .c(new_n103), .o(new_n107));
  oabi12aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .out0(new_n108));
  norp02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  norb02aa1n02x5               g015(.a(new_n110), .b(new_n109), .out0(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norb02aa1n02x5               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nano23aa1n02x4               g021(.a(new_n116), .b(new_n115), .c(new_n114), .d(new_n111), .out0(new_n117));
  aoi112aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(new_n110), .clkout(new_n119));
  xorc02aa1n02x5               g024(.a(\a[7] ), .b(\b[6] ), .out0(new_n120));
  oab012aa1n02x4               g025(.a(new_n109), .b(\a[5] ), .c(\b[4] ), .out0(new_n121));
  nona23aa1n02x4               g026(.a(new_n120), .b(new_n114), .c(new_n121), .d(new_n119), .out0(new_n122));
  nona22aa1n02x4               g027(.a(new_n122), .b(new_n118), .c(new_n112), .out0(new_n123));
  xorc02aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n108), .d(new_n117), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n97), .out0(\s[10] ));
  orn002aa1n02x5               g032(.a(\a[10] ), .b(\b[9] ), .o(new_n128));
  and002aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o(new_n129));
  aoi013aa1n02x4               g034(.a(new_n129), .b(new_n125), .c(new_n128), .d(new_n97), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoi112aa1n02x5               g042(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(\s[12] ));
  nanp02aa1n02x5               g045(.a(new_n117), .b(new_n108), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n112), .b(new_n113), .out0(new_n142));
  oai022aa1n02x5               g047(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n143));
  nano23aa1n02x4               g048(.a(new_n115), .b(new_n142), .c(new_n143), .d(new_n110), .out0(new_n144));
  norp03aa1n02x5               g049(.a(new_n144), .b(new_n118), .c(new_n112), .o1(new_n145));
  nano23aa1n02x4               g050(.a(new_n132), .b(new_n135), .c(new_n136), .d(new_n133), .out0(new_n146));
  nanp03aa1n02x5               g051(.a(new_n146), .b(new_n124), .c(new_n126), .o1(new_n147));
  oaoi03aa1n02x5               g052(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n135), .b(new_n132), .c(new_n136), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  aoi012aa1n02x5               g055(.a(new_n150), .b(new_n146), .c(new_n148), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n147), .c(new_n141), .d(new_n145), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  aoi012aa1n02x5               g060(.a(new_n154), .b(new_n152), .c(new_n155), .o1(new_n156));
  xnrb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  160nm_ficinv00aa1n08x5       g062(.clk(new_n147), .clkout(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n123), .c(new_n108), .d(new_n117), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nona23aa1n02x4               g066(.a(new_n161), .b(new_n155), .c(new_n154), .d(new_n160), .out0(new_n162));
  oai012aa1n02x5               g067(.a(new_n161), .b(new_n160), .c(new_n154), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n162), .c(new_n159), .d(new_n151), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  xorc02aa1n02x5               g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  xorc02aa1n02x5               g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n168), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nano23aa1n02x4               g076(.a(new_n154), .b(new_n160), .c(new_n161), .d(new_n155), .out0(new_n172));
  nanp03aa1n02x5               g077(.a(new_n172), .b(new_n167), .c(new_n168), .o1(new_n173));
  norp02aa1n02x5               g078(.a(new_n147), .b(new_n173), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n123), .c(new_n108), .d(new_n117), .o1(new_n175));
  norp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  160nm_ficinv00aa1n08x5       g081(.clk(new_n176), .clkout(new_n177));
  nanb03aa1n02x5               g082(.a(new_n163), .b(new_n168), .c(new_n167), .out0(new_n178));
  aoi112aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n179), .clkout(new_n180));
  nanp02aa1n02x5               g085(.a(new_n146), .b(new_n148), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n173), .b(new_n181), .c(new_n149), .o1(new_n182));
  nano32aa1n02x4               g087(.a(new_n182), .b(new_n180), .c(new_n178), .d(new_n177), .out0(new_n183));
  xorc02aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n175), .c(new_n183), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  nanp02aa1n02x5               g092(.a(new_n187), .b(new_n186), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n141), .b(new_n145), .o1(new_n189));
  xnrc02aa1n02x5               g094(.a(\b[14] ), .b(\a[15] ), .out0(new_n190));
  xnrc02aa1n02x5               g095(.a(\b[15] ), .b(\a[16] ), .out0(new_n191));
  norp03aa1n02x5               g096(.a(new_n191), .b(new_n190), .c(new_n163), .o1(new_n192));
  norp03aa1n02x5               g097(.a(new_n162), .b(new_n191), .c(new_n190), .o1(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n150), .c(new_n146), .d(new_n148), .o1(new_n194));
  nona32aa1n02x4               g099(.a(new_n194), .b(new_n179), .c(new_n192), .d(new_n176), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n184), .b(new_n195), .c(new_n189), .d(new_n174), .o1(new_n196));
  norp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n196), .c(new_n188), .out0(\s[18] ));
  nanp02aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  nano32aa1n02x4               g106(.a(new_n197), .b(new_n188), .c(new_n198), .d(new_n201), .out0(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n202), .clkout(new_n203));
  aoai13aa1n02x5               g108(.a(new_n198), .b(new_n197), .c(new_n186), .d(new_n187), .o1(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n203), .c(new_n175), .d(new_n183), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  norp02aa1n02x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  aoi112aa1n02x5               g117(.a(new_n208), .b(new_n212), .c(new_n205), .d(new_n209), .o1(new_n213));
  aoai13aa1n02x5               g118(.a(new_n212), .b(new_n208), .c(new_n205), .d(new_n209), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(\s[20] ));
  nano23aa1n02x4               g120(.a(new_n208), .b(new_n210), .c(new_n211), .d(new_n209), .out0(new_n216));
  nanp03aa1n02x5               g121(.a(new_n216), .b(new_n184), .c(new_n199), .o1(new_n217));
  nona23aa1n02x4               g122(.a(new_n211), .b(new_n209), .c(new_n208), .d(new_n210), .out0(new_n218));
  aoi012aa1n02x5               g123(.a(new_n210), .b(new_n208), .c(new_n211), .o1(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n204), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n220), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n217), .c(new_n175), .d(new_n183), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  xorc02aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .out0(new_n226));
  aoi112aa1n02x5               g131(.a(new_n224), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[21] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\a[22] ), .clkout(new_n231));
  xroi22aa1d04x5               g136(.a(new_n230), .b(\b[20] ), .c(new_n231), .d(\b[21] ), .out0(new_n232));
  nanp03aa1n02x5               g137(.a(new_n232), .b(new_n202), .c(new_n216), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n204), .clkout(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n219), .clkout(new_n235));
  aoai13aa1n02x5               g140(.a(new_n232), .b(new_n235), .c(new_n216), .d(new_n234), .o1(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(\b[21] ), .clkout(new_n237));
  oao003aa1n02x5               g142(.a(new_n231), .b(new_n237), .c(new_n224), .carry(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  nanp02aa1n02x5               g144(.a(new_n236), .b(new_n239), .o1(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n233), .c(new_n175), .d(new_n183), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  xorc02aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  xorc02aa1n02x5               g150(.a(\a[24] ), .b(\b[23] ), .out0(new_n246));
  aoi112aa1n02x5               g151(.a(new_n244), .b(new_n246), .c(new_n242), .d(new_n245), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n245), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n248), .b(new_n247), .out0(\s[24] ));
  and002aa1n02x5               g154(.a(new_n246), .b(new_n245), .o(new_n250));
  nanb03aa1n02x5               g155(.a(new_n217), .b(new_n250), .c(new_n232), .out0(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n250), .clkout(new_n252));
  orn002aa1n02x5               g157(.a(\a[23] ), .b(\b[22] ), .o(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .carry(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n252), .c(new_n236), .d(new_n239), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n255), .clkout(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n251), .c(new_n175), .d(new_n183), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  xorc02aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n259), .b(new_n261), .c(new_n257), .d(new_n260), .o1(new_n262));
  aoai13aa1n02x5               g167(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n263), .b(new_n262), .out0(\s[26] ));
  and002aa1n02x5               g169(.a(new_n261), .b(new_n260), .o(new_n265));
  nano22aa1n02x4               g170(.a(new_n233), .b(new_n250), .c(new_n265), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n195), .c(new_n189), .d(new_n174), .o1(new_n267));
  orn002aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .o(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n269));
  aobi12aa1n02x5               g174(.a(new_n269), .b(new_n255), .c(new_n265), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  aobi12aa1n02x5               g179(.a(new_n271), .b(new_n270), .c(new_n267), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  nano22aa1n02x4               g181(.a(new_n275), .b(new_n274), .c(new_n276), .out0(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n266), .clkout(new_n278));
  aoi012aa1n02x5               g183(.a(new_n278), .b(new_n175), .c(new_n183), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n250), .b(new_n238), .c(new_n220), .d(new_n232), .o1(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n265), .clkout(new_n281));
  aoai13aa1n02x5               g186(.a(new_n269), .b(new_n281), .c(new_n280), .d(new_n254), .o1(new_n282));
  oai012aa1n02x5               g187(.a(new_n271), .b(new_n282), .c(new_n279), .o1(new_n283));
  aoi012aa1n02x5               g188(.a(new_n276), .b(new_n283), .c(new_n274), .o1(new_n284));
  norp02aa1n02x5               g189(.a(new_n284), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g190(.a(new_n271), .b(new_n276), .out0(new_n286));
  oai012aa1n02x5               g191(.a(new_n286), .b(new_n282), .c(new_n279), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  aobi12aa1n02x5               g195(.a(new_n286), .b(new_n270), .c(new_n267), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  norp02aa1n02x5               g197(.a(new_n290), .b(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g199(.a(new_n271), .b(new_n289), .c(new_n276), .out0(new_n295));
  oai012aa1n02x5               g200(.a(new_n295), .b(new_n282), .c(new_n279), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[29] ), .b(\a[30] ), .out0(new_n298));
  aoi012aa1n02x5               g203(.a(new_n298), .b(new_n296), .c(new_n297), .o1(new_n299));
  aobi12aa1n02x5               g204(.a(new_n295), .b(new_n270), .c(new_n267), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n297), .c(new_n298), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n299), .b(new_n301), .o1(\s[30] ));
  norb02aa1n02x5               g207(.a(new_n295), .b(new_n298), .out0(new_n303));
  aobi12aa1n02x5               g208(.a(new_n303), .b(new_n270), .c(new_n267), .out0(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n304), .b(new_n305), .c(new_n306), .out0(new_n307));
  oai012aa1n02x5               g212(.a(new_n303), .b(new_n282), .c(new_n279), .o1(new_n308));
  aoi012aa1n02x5               g213(.a(new_n306), .b(new_n308), .c(new_n305), .o1(new_n309));
  norp02aa1n02x5               g214(.a(new_n309), .b(new_n307), .o1(\s[31] ));
  xnrb03aa1n02x5               g215(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g216(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g218(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g219(.a(new_n116), .b(new_n108), .out0(new_n315));
  oai012aa1n02x5               g220(.a(new_n315), .b(\b[4] ), .c(\a[5] ), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g222(.a(new_n120), .b(new_n109), .c(new_n316), .d(new_n110), .o1(new_n318));
  aoi112aa1n02x5               g223(.a(new_n120), .b(new_n109), .c(new_n316), .d(new_n110), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n318), .b(new_n319), .out0(\s[7] ));
  orn002aa1n02x5               g225(.a(\a[7] ), .b(\b[6] ), .o(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n114), .b(new_n318), .c(new_n321), .out0(\s[8] ));
  xnbna2aa1n03x5               g227(.a(new_n124), .b(new_n141), .c(new_n145), .out0(\s[9] ));
endmodule


